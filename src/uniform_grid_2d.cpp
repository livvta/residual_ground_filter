#include "residual_ground_filter/uniform_grid_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace residual_ground_filter
{

void UniformGrid2D::Build(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & point_cloud, const double search_radius)
{
  if (!std::isfinite(search_radius) || search_radius <= 0.0) {
    throw std::invalid_argument("uniform grid search radius must be finite and greater than 0");
  }
  if (
    point_cloud &&
    point_cloud->size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {
    throw std::length_error("uniform grid supports at most INT_MAX points");
  }

  point_cloud_ = point_cloud;
  inverse_cell_size_ = 1.0 / search_radius;
  squared_radius_ = search_radius * search_radius;
  column_count_ = 0U;
  row_count_ = 0U;
  cell_offsets_.clear();
  cell_write_positions_.clear();
  point_indices_.clear();

  if (!point_cloud_ || point_cloud_->empty()) {
    return;
  }

  float min_x = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float min_y = std::numeric_limits<float>::max();
  float max_y = std::numeric_limits<float>::lowest();
  for (const auto & point : point_cloud_->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      throw std::invalid_argument("uniform grid input contains non-finite XY coordinates");
    }
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  origin_x_ = static_cast<double>(min_x);
  origin_y_ = static_cast<double>(min_y);
  const double column_count =
    std::floor((static_cast<double>(max_x) - origin_x_) * inverse_cell_size_) + 1.0;
  const double row_count =
    std::floor((static_cast<double>(max_y) - origin_y_) * inverse_cell_size_) + 1.0;
  const double max_size = static_cast<double>(std::numeric_limits<std::size_t>::max());
  if (
    !std::isfinite(column_count) || !std::isfinite(row_count) || column_count < 1.0 ||
    row_count < 1.0 || column_count > max_size || row_count > max_size)
  {
    throw std::length_error("uniform grid dimensions exceed addressable memory");
  }

  column_count_ = static_cast<std::size_t>(column_count);
  row_count_ = static_cast<std::size_t>(row_count);
  if (
    column_count_ > std::numeric_limits<std::size_t>::max() / row_count_ ||
    column_count_ * row_count_ == std::numeric_limits<std::size_t>::max())
  {
    throw std::length_error("uniform grid cell count exceeds addressable memory");
  }
  const std::size_t cell_count = column_count_ * row_count_;

  cell_offsets_.assign(cell_count + 1U, 0U);
  for (const auto & point : point_cloud_->points) {
    ++cell_offsets_[CellIndex(point.x, point.y) + 1U];
  }
  for (std::size_t i = 1U; i < cell_offsets_.size(); ++i) {
    cell_offsets_[i] += cell_offsets_[i - 1U];
  }

  cell_write_positions_.assign(cell_offsets_.begin(), cell_offsets_.end() - 1);
  point_indices_.resize(point_cloud_->size());
  for (std::size_t i = 0U; i < point_cloud_->size(); ++i) {
    const auto & point = point_cloud_->points[i];
    const std::size_t cell_index = CellIndex(point.x, point.y);
    point_indices_[cell_write_positions_[cell_index]++] = static_cast<int>(i);
  }
}

int UniformGrid2D::RadiusSearch(
  const std::size_t query_index, std::vector<int> & neighbor_indices,
  const unsigned int max_neighbors) const
{
  neighbor_indices.clear();
  if (!point_cloud_ || query_index >= point_cloud_->size() || max_neighbors == 0U) {
    return 0;
  }

  const auto & query = point_cloud_->points[query_index];
  const std::size_t query_cell = CellIndex(query.x, query.y);
  const std::size_t query_column = query_cell % column_count_;
  const std::size_t query_row = query_cell / column_count_;
  const std::size_t first_column = query_column == 0U ? 0U : query_column - 1U;
  const std::size_t last_column = std::min(query_column + 1U, column_count_ - 1U);
  const std::size_t first_row = query_row == 0U ? 0U : query_row - 1U;
  const std::size_t last_row = std::min(query_row + 1U, row_count_ - 1U);

  for (std::size_t row = first_row; row <= last_row; ++row) {
    for (std::size_t column = first_column; column <= last_column; ++column) {
      const std::size_t cell_index = row * column_count_ + column;
      for (
        std::size_t offset = cell_offsets_[cell_index];
        offset < cell_offsets_[cell_index + 1U]; ++offset)
      {
        const int candidate_index = point_indices_[offset];
        const auto & candidate = point_cloud_->points[static_cast<std::size_t>(candidate_index)];
        const double dx = static_cast<double>(candidate.x) - static_cast<double>(query.x);
        const double dy = static_cast<double>(candidate.y) - static_cast<double>(query.y);
        if (dx * dx + dy * dy <= squared_radius_) {
          neighbor_indices.push_back(candidate_index);
          if (neighbor_indices.size() >= max_neighbors) {
            return static_cast<int>(neighbor_indices.size());
          }
        }
      }
    }
  }

  return static_cast<int>(neighbor_indices.size());
}

std::size_t UniformGrid2D::CellIndex(const float x, const float y) const
{
  const double raw_column = std::floor((static_cast<double>(x) - origin_x_) * inverse_cell_size_);
  const double raw_row = std::floor((static_cast<double>(y) - origin_y_) * inverse_cell_size_);
  const std::size_t column = std::min(
    static_cast<std::size_t>(std::max(0.0, raw_column)), column_count_ - 1U);
  const std::size_t row = std::min(
    static_cast<std::size_t>(std::max(0.0, raw_row)), row_count_ - 1U);
  return row * column_count_ + column;
}

}  // namespace residual_ground_filter
