#ifndef RADIUS_SEARCH_2D_OUTLIER_FILTER__POINT_PRE_FILTER_HPP_
#define RADIUS_SEARCH_2D_OUTLIER_FILTER__POINT_PRE_FILTER_HPP_

#include <cmath>
#include <limits>

#include <pcl/point_types.h>

namespace radius_search_2d_outlier_filter
{

inline constexpr double kUnlimitedPreFilterMaxZ = std::numeric_limits<double>::max();

inline bool IsValidInputPoint(
  const pcl::PointXYZ & point, const bool remove_zero_points)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return false;
  }

  constexpr float zero_epsilon = 1.0e-6F;
  return !remove_zero_points ||
         std::abs(point.x) > zero_epsilon ||
         std::abs(point.y) > zero_epsilon ||
         std::abs(point.z) > zero_epsilon;
}

inline bool ShouldProjectForNeighborSearch(const pcl::PointXYZ & point, const double max_z)
{
  return static_cast<double>(point.z) <= max_z;
}

}  // namespace radius_search_2d_outlier_filter

#endif  // RADIUS_SEARCH_2D_OUTLIER_FILTER__POINT_PRE_FILTER_HPP_
