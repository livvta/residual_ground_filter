#include "radius_search_2d_outlier_filter/filter.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <utility>

#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace radius_search_2d_outlier_filter
{

Filter::Filter(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  input_frame_(declare_parameter<std::string>("input_frame", "")),
  output_frame_(declare_parameter<std::string>("output_frame", "")),
  transform_timeout_sec_(declare_parameter<double>("transform_timeout_sec", 0.1)),
  max_queue_size_(declare_parameter<int>("max_queue_size", 5)),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_)
{
  if (max_queue_size_ < 1) {
    throw std::invalid_argument("max_queue_size must be at least 1");
  }
  if (transform_timeout_sec_ < 0.0) {
    throw std::invalid_argument("transform_timeout_sec must not be negative");
  }

  auto qos = rclcpp::SensorDataQoS().keep_last(static_cast<std::size_t>(max_queue_size_));

  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  output_publisher_ = create_publisher<PointCloud2>("output", qos, publisher_options);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();
  input_subscription_ = create_subscription<PointCloud2>(
    "input", qos, std::bind(&Filter::pointCloudCallback, this, std::placeholders::_1),
    subscription_options);
}

bool Filter::validatePointCloud(const PointCloud2 & input)
{
  if (input.point_step == 0U) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Ignoring cloud with point_step=0");
    return false;
  }
  const auto point_count = static_cast<std::size_t>(input.width) * input.height;
  if (input.data.size() < point_count * input.point_step) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring malformed cloud: data is smaller than its layout");
    return false;
  }

  const auto has_float32_field = [&input](const char * name) {
      return std::any_of(input.fields.begin(), input.fields.end(), [name](const auto & field) {
        return field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32;
      });
    };
  if (!has_float32_field("x") || !has_float32_field("y") || !has_float32_field("z")) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Ignoring cloud without FLOAT32 x/y/z fields");
    return false;
  }
  return true;
}

bool Filter::transformPointCloud(
  const std::string & target_frame, const PointCloud2 & input, PointCloud2 & output)
{
  if (target_frame.empty() || target_frame == input.header.frame_id) {
    output = input;
    return true;
  }

  try {
    const auto transform = tf_buffer_.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp,
      rclcpp::Duration::from_seconds(transform_timeout_sec_));
    tf2::doTransform(input, output, transform);
    output.header.stamp = input.header.stamp;
    output.header.frame_id = target_frame;
    return true;
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Point cloud transform %s -> %s failed: %s",
      input.header.frame_id.c_str(), target_frame.c_str(), error.what());
    return false;
  }
}

void Filter::pointCloudCallback(const PointCloud2::ConstSharedPtr input)
{
  if (!validatePointCloud(*input)) {
    return;
  }

  const std::string original_frame = input->header.frame_id;
  PointCloud2::ConstSharedPtr processing_input = input;
  auto transformed_input = std::make_shared<PointCloud2>();
  if (!input_frame_.empty() && input_frame_ != original_frame) {
    if (!transformPointCloud(input_frame_, *input, *transformed_input)) {
      return;
    }
    processing_input = transformed_input;
  }

  PointCloud2 filtered;
  filter(processing_input, filtered);
  filtered.header.stamp = input->header.stamp;

  const std::string target_frame = output_frame_.empty() ? original_frame : output_frame_;
  if (target_frame.empty() || filtered.header.frame_id == target_frame) {
    filtered.header.frame_id = target_frame.empty() ? filtered.header.frame_id : target_frame;
    output_publisher_->publish(std::move(filtered));
    return;
  }

  PointCloud2 transformed_output;
  if (transformPointCloud(target_frame, filtered, transformed_output)) {
    transformed_output.header.stamp = input->header.stamp;
    output_publisher_->publish(std::move(transformed_output));
  }
}

}  // namespace radius_search_2d_outlier_filter
