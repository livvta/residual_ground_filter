#ifndef RADIUS_SEARCH_2D_OUTLIER_FILTER__UNIFORM_GRID_2D_HPP_
#define RADIUS_SEARCH_2D_OUTLIER_FILTER__UNIFORM_GRID_2D_HPP_

#include <cstddef>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace radius_search_2d_outlier_filter
{

// A dense uniform XY grid. Each bucket occupies a contiguous range in point_indices_.
class UniformGrid2D
{
public:
  void Build(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & point_cloud, double search_radius);

  // Returns at most max_neighbors exact circular neighbors, including the query point.
  // If fewer than max_neighbors are found, all neighbors inside the radius are returned.
  int RadiusSearch(
    std::size_t query_index, std::vector<int> & neighbor_indices,
    unsigned int max_neighbors) const;

private:
  std::size_t CellIndex(float x, float y) const;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud_;
  double origin_x_{0.0};
  double origin_y_{0.0};
  double inverse_cell_size_{0.0};
  double squared_radius_{0.0};
  std::size_t column_count_{0U};
  std::size_t row_count_{0U};
  std::vector<std::size_t> cell_offsets_;
  std::vector<std::size_t> cell_write_positions_;
  std::vector<int> point_indices_;
};

}  // namespace radius_search_2d_outlier_filter

#endif  // RADIUS_SEARCH_2D_OUTLIER_FILTER__UNIFORM_GRID_2D_HPP_
