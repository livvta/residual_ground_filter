#ifndef RESIDUAL_GROUND_FILTER__VERTICAL_DISTRIBUTION_HPP_
#define RESIDUAL_GROUND_FILTER__VERTICAL_DISTRIBUTION_HPP_

#include <cstddef>
#include <vector>

namespace residual_ground_filter
{

struct VerticalRescueParameters
{
  bool enabled{true};
  float min_z_span{0.25F};
  float separation_threshold{0.10F};
  int min_separated_points{3};
  float bin_size{0.10F};
  float z_window{0.80F};
  int min_occupied_bins{3};

  // Derived once at startup or parameter update, not once per point.
  float inverse_bin_size{10.0F};
  int bin_count{16};
};

void ValidateAndFinalizeVerticalRescueParameters(
  VerticalRescueParameters & parameters, int min_neighbors);

bool HasVerticalZDistribution(
  std::size_t query_index, int neighbor_count, const std::vector<int> & neighbor_indices,
  const std::vector<float> & z_values, const VerticalRescueParameters & parameters);

}  // namespace residual_ground_filter

#endif  // RESIDUAL_GROUND_FILTER__VERTICAL_DISTRIBUTION_HPP_
