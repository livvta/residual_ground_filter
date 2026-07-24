#include "residual_ground_filter/residual_ground_filter_node.hpp"

#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace residual_ground_filter
{
namespace
{

std::string DescribePreFilterMaxZ(const double max_z)
{
  return max_z == kUnlimitedPreFilterMaxZ ? "unlimited" : std::to_string(max_z) + " m";
}

}  // namespace

ResidualGroundFilterComponent::ResidualGroundFilterComponent(
  const rclcpp::NodeOptions & options)
: PointCloudFilterBase("residual_ground_filter", options),
  min_neighbors_(declare_parameter<int>("min_neighbors", 5)),
  search_radius_(declare_parameter<double>("search_radius", 0.2)),
  remove_zero_points_(declare_parameter<bool>("remove_zero_points", false)),
  pre_filter_max_z_(
    declare_parameter<double>("pre_filter.max_z", kUnlimitedPreFilterMaxZ))
{
  if (min_neighbors_ < 1) {
    throw std::invalid_argument("min_neighbors must be at least 1");
  }
  if (!std::isfinite(search_radius_) || search_radius_ <= 0.0) {
    throw std::invalid_argument("search_radius must be finite and greater than 0");
  }
  if (!std::isfinite(pre_filter_max_z_)) {
    throw std::invalid_argument("pre_filter.max_z must be finite");
  }
  deletion_z_.enabled = declare_parameter<bool>("deletion_z.enabled", deletion_z_.enabled);
  deletion_z_.min_z = static_cast<float>(
    declare_parameter<double>("deletion_z.min", deletion_z_.min_z));
  deletion_z_.max_z = static_cast<float>(
    declare_parameter<double>("deletion_z.max", deletion_z_.max_z));
  if (
    !std::isfinite(deletion_z_.min_z) || !std::isfinite(deletion_z_.max_z) ||
    deletion_z_.min_z >= deletion_z_.max_z)
  {
    throw std::invalid_argument(
            "deletion_z bounds must be finite and deletion_z.min must be below deletion_z.max");
  }
  vertical_rescue_.enabled =
    declare_parameter<bool>("vertical_rescue.enabled", vertical_rescue_.enabled);
  vertical_rescue_.min_z_span = static_cast<float>(
    declare_parameter<double>("vertical_rescue.min_z_span", vertical_rescue_.min_z_span));
  vertical_rescue_.separation_threshold = static_cast<float>(declare_parameter<double>(
      "vertical_rescue.separation_threshold", vertical_rescue_.separation_threshold));
  vertical_rescue_.min_separated_points = static_cast<int>(declare_parameter<int64_t>(
      "vertical_rescue.min_separated_points", vertical_rescue_.min_separated_points));
  vertical_rescue_.bin_size = static_cast<float>(
    declare_parameter<double>("vertical_rescue.bin_size", vertical_rescue_.bin_size));
  vertical_rescue_.z_window = static_cast<float>(
    declare_parameter<double>("vertical_rescue.z_window", vertical_rescue_.z_window));
  vertical_rescue_.min_occupied_bins = static_cast<int>(declare_parameter<int64_t>(
      "vertical_rescue.min_occupied_bins", vertical_rescue_.min_occupied_bins));
  ValidateAndFinalizeVerticalRescueParameters(vertical_rescue_, min_neighbors_);

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(
      &ResidualGroundFilterComponent::onParameterChange, this,
      std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Residual ground filter: radius=%.3f m min_neighbors=%d remove_zero_points=%s "
    "pre_filter.max_z=%s deletion_z=%s[%.2f, %.2f] vertical_rescue=%s",
    search_radius_, min_neighbors_, remove_zero_points_ ? "true" : "false",
    DescribePreFilterMaxZ(pre_filter_max_z_).c_str(),
    deletion_z_.enabled ? "" : "disabled ", deletion_z_.min_z, deletion_z_.max_z,
    vertical_rescue_.enabled ? "true" : "false");
}

rcl_interfaces::msg::SetParametersResult
ResidualGroundFilterComponent::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto next_min_neighbors = min_neighbors_;
  auto next_search_radius = search_radius_;
  auto next_remove_zero_points = remove_zero_points_;
  auto next_pre_filter_max_z = pre_filter_max_z_;
  auto next_deletion_z = deletion_z_;
  auto next_vertical_rescue = vertical_rescue_;

  for (const auto & parameter : parameters) {
    try {
      if (parameter.get_name() == "min_neighbors") {
        next_min_neighbors = static_cast<int>(parameter.as_int());
      } else if (parameter.get_name() == "search_radius") {
        next_search_radius = parameter.as_double();
      } else if (parameter.get_name() == "remove_zero_points") {
        next_remove_zero_points = parameter.as_bool();
      } else if (parameter.get_name() == "pre_filter.max_z") {
        next_pre_filter_max_z = parameter.as_double();
      } else if (parameter.get_name() == "deletion_z.enabled") {
        next_deletion_z.enabled = parameter.as_bool();
      } else if (parameter.get_name() == "deletion_z.min") {
        next_deletion_z.min_z = static_cast<float>(parameter.as_double());
      } else if (parameter.get_name() == "deletion_z.max") {
        next_deletion_z.max_z = static_cast<float>(parameter.as_double());
      } else if (parameter.get_name() == "vertical_rescue.enabled") {
        next_vertical_rescue.enabled = parameter.as_bool();
      } else if (parameter.get_name() == "vertical_rescue.min_z_span") {
        next_vertical_rescue.min_z_span = static_cast<float>(parameter.as_double());
      } else if (parameter.get_name() == "vertical_rescue.separation_threshold") {
        next_vertical_rescue.separation_threshold = static_cast<float>(parameter.as_double());
      } else if (parameter.get_name() == "vertical_rescue.min_separated_points") {
        next_vertical_rescue.min_separated_points = static_cast<int>(parameter.as_int());
      } else if (parameter.get_name() == "vertical_rescue.bin_size") {
        next_vertical_rescue.bin_size = static_cast<float>(parameter.as_double());
      } else if (parameter.get_name() == "vertical_rescue.z_window") {
        next_vertical_rescue.z_window = static_cast<float>(parameter.as_double());
      } else if (parameter.get_name() == "vertical_rescue.min_occupied_bins") {
        next_vertical_rescue.min_occupied_bins = static_cast<int>(parameter.as_int());
      } else if (
        parameter.get_name() == "input_frame" || parameter.get_name() == "output_frame" ||
        parameter.get_name() == "transform_timeout_sec" ||
        parameter.get_name() == "max_queue_size")
      {
        return rcl_interfaces::msg::SetParametersResult()
          .set__successful(false)
          .set__reason(parameter.get_name() + " requires a node restart");
      }
    } catch (const rclcpp::ParameterTypeException & error) {
      return rcl_interfaces::msg::SetParametersResult()
        .set__successful(false)
        .set__reason(error.what());
    }
  }

  if (next_min_neighbors < 1) {
    return rcl_interfaces::msg::SetParametersResult()
      .set__successful(false)
      .set__reason("min_neighbors must be at least 1");
  }
  if (!std::isfinite(next_search_radius) || next_search_radius <= 0.0) {
    return rcl_interfaces::msg::SetParametersResult()
      .set__successful(false)
      .set__reason("search_radius must be finite and greater than 0");
  }
  if (!std::isfinite(next_pre_filter_max_z)) {
    return rcl_interfaces::msg::SetParametersResult()
      .set__successful(false)
      .set__reason("pre_filter.max_z must be finite");
  }
  if (
    !std::isfinite(next_deletion_z.min_z) || !std::isfinite(next_deletion_z.max_z) ||
    next_deletion_z.min_z >= next_deletion_z.max_z)
  {
    return rcl_interfaces::msg::SetParametersResult()
      .set__successful(false)
      .set__reason(
      "deletion_z bounds must be finite and deletion_z.min must be below deletion_z.max");
  }
  try {
    ValidateAndFinalizeVerticalRescueParameters(next_vertical_rescue, next_min_neighbors);
  } catch (const std::invalid_argument & error) {
    return rcl_interfaces::msg::SetParametersResult()
      .set__successful(false)
      .set__reason(error.what());
  }

  min_neighbors_ = next_min_neighbors;
  search_radius_ = next_search_radius;
  remove_zero_points_ = next_remove_zero_points;
  pre_filter_max_z_ = next_pre_filter_max_z;
  deletion_z_ = next_deletion_z;
  vertical_rescue_ = next_vertical_rescue;
  RCLCPP_INFO(
    get_logger(),
    "Updated: radius=%.3f m min_neighbors=%d remove_zero_points=%s "
    "pre_filter.max_z=%s deletion_z=%s[%.2f, %.2f] vertical_rescue=%s",
    search_radius_, min_neighbors_, remove_zero_points_ ? "true" : "false",
    DescribePreFilterMaxZ(pre_filter_max_z_).c_str(),
    deletion_z_.enabled ? "" : "disabled ", deletion_z_.min_z, deletion_z_.max_z,
    vertical_rescue_.enabled ? "true" : "false");
  return rcl_interfaces::msg::SetParametersResult().set__successful(true);
}

void ResidualGroundFilterComponent::filter(
  const PointCloud2::ConstSharedPtr & input, PointCloud2 & output)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto xyz_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*input, *xyz_cloud);

  auto xy_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  xy_cloud->reserve(xyz_cloud->size());
  std::vector<std::size_t> source_indices;
  source_indices.reserve(xyz_cloud->size());
  std::vector<float> z_values;
  z_values.reserve(xyz_cloud->size());
  std::vector<bool> keep_points(xyz_cloud->size(), false);

  for (std::size_t i = 0; i < xyz_cloud->size(); ++i) {
    const auto & point = xyz_cloud->points[i];
    if (!IsValidInputPoint(point, remove_zero_points_)) {
      continue;
    }
    if (!ShouldProjectForNeighborSearch(point, pre_filter_max_z_)) {
      // High points bypass the 2D index and classification, but remain in the output.
      keep_points[i] = true;
      continue;
    }
    xy_cloud->push_back(pcl::PointXYZ{point.x, point.y, 0.0F});
    source_indices.push_back(i);
    z_values.push_back(point.z);
  }

  if (!xy_cloud->empty()) {
    // The classification only needs to know whether the threshold is reached. Bounding the
    // result count avoids enumerating every neighbor in dense regions without changing output.
    const auto max_neighbors = static_cast<unsigned int>(min_neighbors_);
    std::vector<int> neighbor_indices;
    neighbor_indices.reserve(max_neighbors);
    grid_.Build(xy_cloud, search_radius_);

    for (std::size_t i = 0; i < xy_cloud->size(); ++i) {
      const float query_z = z_values[i];
      if (
        deletion_z_.enabled &&
        (query_z < deletion_z_.min_z || query_z > deletion_z_.max_z))
      {
        // Protect points outside the ground-height band and avoid their neighbor search.
        keep_points[source_indices[i]] = true;
        continue;
      }

      const int count = grid_.RadiusSearch(i, neighbor_indices, max_neighbors);
      if (
        count >= min_neighbors_ ||
        HasVerticalZDistribution(
          i, count, neighbor_indices, z_values, vertical_rescue_))
      {
        keep_points[source_indices[i]] = true;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  filtered_cloud.reserve(xyz_cloud->size());
  for (std::size_t i = 0; i < xyz_cloud->size(); ++i) {
    if (keep_points[i]) {
      filtered_cloud.push_back(xyz_cloud->points[i]);
    }
  }

  filtered_cloud.width = static_cast<std::uint32_t>(filtered_cloud.size());
  filtered_cloud.height = 1U;
  filtered_cloud.is_dense = true;
  pcl::toROSMsg(filtered_cloud, output);
  output.header = input->header;
}

}  // namespace residual_ground_filter

RCLCPP_COMPONENTS_REGISTER_NODE(
  residual_ground_filter::ResidualGroundFilterComponent)
