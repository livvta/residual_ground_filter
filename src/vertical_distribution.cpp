#include "radius_search_2d_outlier_filter/vertical_distribution.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace radius_search_2d_outlier_filter
{
namespace
{

int CountSetBits(std::uint64_t value)
{
  int count = 0;
  while (value != 0U) {
    value &= value - 1U;
    ++count;
  }
  return count;
}

}  // namespace

void ValidateAndFinalizeVerticalRescueParameters(
  VerticalRescueParameters & parameters, const int min_neighbors)
{
  if (!std::isfinite(parameters.min_z_span) || parameters.min_z_span <= 0.0F) {
    throw std::invalid_argument("vertical_rescue.min_z_span must be finite and greater than 0");
  }
  if (
    !std::isfinite(parameters.separation_threshold) ||
    parameters.separation_threshold < 0.0F)
  {
    throw std::invalid_argument(
            "vertical_rescue.separation_threshold must be finite and non-negative");
  }
  if (parameters.min_separated_points < 1) {
    throw std::invalid_argument("vertical_rescue.min_separated_points must be at least 1");
  }
  if (!std::isfinite(parameters.bin_size) || parameters.bin_size <= 0.0F) {
    throw std::invalid_argument("vertical_rescue.bin_size must be finite and greater than 0");
  }
  if (!std::isfinite(parameters.z_window) || parameters.z_window <= 0.0F) {
    throw std::invalid_argument("vertical_rescue.z_window must be finite and greater than 0");
  }
  if (parameters.min_occupied_bins < 1) {
    throw std::invalid_argument("vertical_rescue.min_occupied_bins must be at least 1");
  }

  const double bins = std::ceil(
    (2.0 * static_cast<double>(parameters.z_window)) /
    static_cast<double>(parameters.bin_size));
  if (!std::isfinite(bins) || bins < 1.0 || bins > 64.0) {
    throw std::invalid_argument(
            "vertical_rescue requires between 1 and 64 Z bins; adjust z_window or bin_size");
  }
  parameters.bin_count = static_cast<int>(bins);
  parameters.inverse_bin_size = 1.0F / parameters.bin_size;

  if (parameters.min_occupied_bins > parameters.bin_count) {
    throw std::invalid_argument(
            "vertical_rescue.min_occupied_bins must not exceed the available Z bins");
  }
  if (
    parameters.enabled &&
    (parameters.min_separated_points >= min_neighbors ||
    parameters.min_occupied_bins > min_neighbors))
  {
    throw std::invalid_argument(
            "vertical rescue thresholds cannot be reached below min_neighbors");
  }
}

bool HasVerticalZDistribution(
  const std::size_t query_index, const int neighbor_count,
  const std::vector<int> & neighbor_indices, const std::vector<float> & z_values,
  const VerticalRescueParameters & parameters)
{
  if (
    !parameters.enabled || query_index >= z_values.size() || neighbor_count <= 0 ||
    static_cast<std::size_t>(neighbor_count) > neighbor_indices.size())
  {
    return false;
  }

  const float query_z = z_values[query_index];
  float min_z = query_z;
  float max_z = query_z;
  int separated_points = 0;
  std::uint64_t occupied_bins = 0U;

  for (int j = 0; j < neighbor_count; ++j) {
    const int neighbor_index = neighbor_indices[static_cast<std::size_t>(j)];
    if (neighbor_index < 0 || static_cast<std::size_t>(neighbor_index) >= z_values.size()) {
      return false;
    }

    const float z = z_values[static_cast<std::size_t>(neighbor_index)];
    min_z = std::min(min_z, z);
    max_z = std::max(max_z, z);

    const float dz = z - query_z;
    if (std::abs(dz) >= parameters.separation_threshold) {
      ++separated_points;
    }

    // Check the window before conversion so negative values are never truncated into bin zero.
    if (dz >= -parameters.z_window && dz < parameters.z_window) {
      const int bin = static_cast<int>(
        (dz + parameters.z_window) * parameters.inverse_bin_size);
      if (bin >= 0 && bin < parameters.bin_count) {
        occupied_bins |= std::uint64_t{1} << static_cast<unsigned int>(bin);
      }
    }
  }

  return max_z - min_z >= parameters.min_z_span &&
         separated_points >= parameters.min_separated_points &&
         CountSetBits(occupied_bins) >= parameters.min_occupied_bins;
}

}  // namespace radius_search_2d_outlier_filter
