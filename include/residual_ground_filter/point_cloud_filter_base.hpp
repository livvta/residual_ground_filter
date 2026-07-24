#ifndef RESIDUAL_GROUND_FILTER__POINT_CLOUD_FILTER_BASE_HPP_
#define RESIDUAL_GROUND_FILTER__POINT_CLOUD_FILTER_BASE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace residual_ground_filter
{

// ROS 2 base node for validated, optionally transformed PointCloud2 filtering.
class PointCloudFilterBase : public rclcpp::Node
{
public:
  explicit PointCloudFilterBase(
    const std::string & node_name, const rclcpp::NodeOptions & options);

protected:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  virtual void filter(const PointCloud2::ConstSharedPtr & input, PointCloud2 & output) = 0;

private:
  void pointCloudCallback(const PointCloud2::ConstSharedPtr input);
  bool validatePointCloud(const PointCloud2 & input);
  bool transformPointCloud(
    const std::string & target_frame, const PointCloud2 & input, PointCloud2 & output);

  std::string input_frame_;
  std::string output_frame_;
  double transform_timeout_sec_;
  int max_queue_size_;

  rclcpp::Subscription<PointCloud2>::SharedPtr input_subscription_;
  rclcpp::Publisher<PointCloud2>::SharedPtr output_publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace residual_ground_filter

#endif  // RESIDUAL_GROUND_FILTER__POINT_CLOUD_FILTER_BASE_HPP_
