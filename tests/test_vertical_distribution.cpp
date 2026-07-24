#include "residual_ground_filter/vertical_distribution.hpp"

#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

namespace residual_ground_filter
{
namespace
{

VerticalRescueParameters DefaultParameters()
{
  VerticalRescueParameters parameters;
  ValidateAndFinalizeVerticalRescueParameters(parameters, 5);
  return parameters;
}

TEST(VerticalDistribution, KeepsSparseVerticalStructure)
{
  const auto parameters = DefaultParameters();
  const std::vector<float> z_values{0.0F, 0.12F, 0.28F, 0.40F};
  const std::vector<int> neighbors{0, 1, 2, 3};

  EXPECT_TRUE(HasVerticalZDistribution(0, 4, neighbors, z_values, parameters));
}

TEST(VerticalDistribution, RemovesSparsePlanarStructure)
{
  const auto parameters = DefaultParameters();
  const std::vector<float> z_values{0.0F, 0.01F, -0.01F, 0.02F};
  const std::vector<int> neighbors{0, 1, 2, 3};

  EXPECT_FALSE(HasVerticalZDistribution(0, 4, neighbors, z_values, parameters));
}

TEST(VerticalDistribution, RequiresMultipleHeightBins)
{
  const auto parameters = DefaultParameters();
  const std::vector<float> z_values{0.0F, 0.30F, 0.31F, 0.32F};
  const std::vector<int> neighbors{0, 1, 2, 3};

  EXPECT_FALSE(HasVerticalZDistribution(0, 4, neighbors, z_values, parameters));
}

TEST(VerticalDistribution, DisabledNeverRescues)
{
  auto parameters = DefaultParameters();
  parameters.enabled = false;
  const std::vector<float> z_values{0.0F, 0.12F, 0.28F, 0.40F};
  const std::vector<int> neighbors{0, 1, 2, 3};

  EXPECT_FALSE(HasVerticalZDistribution(0, 4, neighbors, z_values, parameters));
}

TEST(VerticalDistribution, RejectsUnreachableThresholds)
{
  auto parameters = DefaultParameters();
  parameters.min_separated_points = 5;

  EXPECT_THROW(ValidateAndFinalizeVerticalRescueParameters(parameters, 5), std::invalid_argument);
}

TEST(VerticalDistribution, SupportsFortyHeightBinsWithoutAllocation)
{
  auto parameters = DefaultParameters();
  parameters.z_window = 2.0F;
  parameters.bin_size = 0.10F;

  EXPECT_NO_THROW(ValidateAndFinalizeVerticalRescueParameters(parameters, 5));
  EXPECT_EQ(parameters.bin_count, 40);
}

}  // namespace
}  // namespace residual_ground_filter
