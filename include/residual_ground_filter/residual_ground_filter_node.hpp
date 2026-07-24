#ifndef RESIDUAL_GROUND_FILTER__RESIDUAL_GROUND_FILTER_NODE_HPP_
#define RESIDUAL_GROUND_FILTER__RESIDUAL_GROUND_FILTER_NODE_HPP_

#include "residual_ground_filter/point_cloud_filter_base.hpp"
#include "residual_ground_filter/point_pre_filter.hpp"
#include "residual_ground_filter/uniform_grid_2d.hpp"
#include "residual_ground_filter/vertical_distribution.hpp"

#include <memory>
#include <mutex>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace residual_ground_filter
{

struct CandidateZRange
{
  bool enabled{true};
  float min_z{-2.50F};
  float max_z{-1.60F};
};

class ResidualGroundFilterComponent : public PointCloudFilterBase
{
public:
  explicit ResidualGroundFilterComponent(const rclcpp::NodeOptions & options);

protected:
  void filter(const PointCloud2::ConstSharedPtr & input, PointCloud2 & output) override;

private:
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & parameters);

  std::mutex mutex_;
  int min_neighbors_;
  double search_radius_;
  bool remove_zero_points_;
  double pre_filter_max_z_;
  CandidateZRange deletion_z_;
  VerticalRescueParameters vertical_rescue_;
  UniformGrid2D grid_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace residual_ground_filter

#endif  // RESIDUAL_GROUND_FILTER__RESIDUAL_GROUND_FILTER_NODE_HPP_
