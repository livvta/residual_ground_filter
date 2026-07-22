#ifndef RADIUS_SEARCH_2D_OUTLIER_FILTER__FILTER_HPP_
#define RADIUS_SEARCH_2D_OUTLIER_FILTER__FILTER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace radius_search_2d_outlier_filter
{

// Relevant standalone subset of Autoware's pointcloud_preprocessor Filter base.
class Filter : public rclcpp::Node
{
public:
  explicit Filter(const std::string & node_name, const rclcpp::NodeOptions & options);

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

}  // namespace radius_search_2d_outlier_filter

#endif  // RADIUS_SEARCH_2D_OUTLIER_FILTER__FILTER_HPP_
