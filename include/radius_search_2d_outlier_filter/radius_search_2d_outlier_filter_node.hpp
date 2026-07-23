#ifndef RADIUS_SEARCH_2D_OUTLIER_FILTER__RADIUS_SEARCH_2D_OUTLIER_FILTER_NODE_HPP_
#define RADIUS_SEARCH_2D_OUTLIER_FILTER__RADIUS_SEARCH_2D_OUTLIER_FILTER_NODE_HPP_

#include "radius_search_2d_outlier_filter/filter.hpp"
#include "radius_search_2d_outlier_filter/point_pre_filter.hpp"
#include "radius_search_2d_outlier_filter/uniform_grid_2d.hpp"
#include "radius_search_2d_outlier_filter/vertical_distribution.hpp"

#include <memory>
#include <mutex>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace radius_search_2d_outlier_filter
{

struct DeletionZRange
{
  bool enabled{true};
  float min_z{-2.50F};
  float max_z{-1.60F};
};

class RadiusSearch2DOutlierFilterComponent : public Filter
{
public:
  explicit RadiusSearch2DOutlierFilterComponent(const rclcpp::NodeOptions & options);

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
  DeletionZRange deletion_z_;
  VerticalRescueParameters vertical_rescue_;
  UniformGrid2D grid_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace radius_search_2d_outlier_filter

#endif  // RADIUS_SEARCH_2D_OUTLIER_FILTER__RADIUS_SEARCH_2D_OUTLIER_FILTER_NODE_HPP_
