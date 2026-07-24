#include "residual_ground_filter/point_pre_filter.hpp"
#include "residual_ground_filter/uniform_grid_2d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace residual_ground_filter
{
namespace
{

pcl::PointCloud<pcl::PointXYZ>::Ptr MakeCloud(
  const std::vector<std::pair<float, float>> & coordinates)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(coordinates.size());
  for (const auto & coordinate : coordinates) {
    cloud->push_back(pcl::PointXYZ{coordinate.first, coordinate.second, 0.0F});
  }
  return cloud;
}

std::vector<int> BruteForceNeighbors(
  const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::size_t query_index,
  const double radius)
{
  std::vector<int> result;
  const auto & query = cloud.points[query_index];
  const double squared_radius = radius * radius;
  for (std::size_t i = 0U; i < cloud.size(); ++i) {
    const double dx = static_cast<double>(cloud.points[i].x) - query.x;
    const double dy = static_cast<double>(cloud.points[i].y) - query.y;
    if (dx * dx + dy * dy <= squared_radius) {
      result.push_back(static_cast<int>(i));
    }
  }
  return result;
}

TEST(UniformGrid2D, MatchesExactCircleAcrossNegativeCoordinatesAndCellBoundaries)
{
  constexpr double radius = 1.0;
  const auto cloud = MakeCloud({
      {-2.0F, 0.0F}, {-1.0F, 0.0F}, {0.0F, 0.0F}, {1.0F, 0.0F},
      {0.8F, 0.8F}, {0.0F, 1.0F}, {0.0F, -1.0F}});
  UniformGrid2D grid;
  grid.Build(cloud, radius);

  std::vector<int> actual;
  const int count = grid.RadiusSearch(2U, actual, static_cast<unsigned int>(cloud->size()));
  auto expected = BruteForceNeighbors(*cloud, 2U, radius);
  std::sort(actual.begin(), actual.end());
  std::sort(expected.begin(), expected.end());

  EXPECT_EQ(count, static_cast<int>(expected.size()));
  EXPECT_EQ(actual, expected);
}

TEST(UniformGrid2D, StopsAtRequestedNeighborThreshold)
{
  constexpr double radius = 1.0;
  const auto cloud = MakeCloud({
      {0.0F, 0.0F}, {0.1F, 0.0F}, {-0.1F, 0.0F}, {0.0F, 0.1F}, {0.0F, -0.1F}});
  UniformGrid2D grid;
  grid.Build(cloud, radius);

  std::vector<int> neighbors;
  EXPECT_EQ(grid.RadiusSearch(0U, neighbors, 3U), 3);
  ASSERT_EQ(neighbors.size(), 3U);
  for (const int index : neighbors) {
    const auto & point = cloud->points[static_cast<std::size_t>(index)];
    EXPECT_LE(
      static_cast<double>(point.x) * point.x + static_cast<double>(point.y) * point.y,
      radius * radius);
  }
}

TEST(UniformGrid2D, HeightPreFilterExcludesCeilingPointsFromNeighborCount)
{
  constexpr double max_z = 1.0;
  constexpr double radius = 0.5;
  const std::vector<pcl::PointXYZ> source_points{
    {0.0F, 0.0F, -2.0F},
    {0.05F, 0.0F, 2.5F},
    {-0.05F, 0.0F, 2.5F},
    {0.0F, 0.05F, 2.5F},
    {0.0F, -0.05F, 2.5F},
  };

  auto projected_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<pcl::PointXYZ> passthrough_points;
  for (const auto & point : source_points) {
    ASSERT_TRUE(IsValidInputPoint(point, false));
    if (ShouldProjectForNeighborSearch(point, max_z)) {
      projected_cloud->push_back(pcl::PointXYZ{point.x, point.y, 0.0F});
    } else {
      passthrough_points.push_back(point);
    }
  }

  ASSERT_EQ(projected_cloud->size(), 1U);
  ASSERT_EQ(passthrough_points.size(), 4U);
  UniformGrid2D grid;
  grid.Build(projected_cloud, radius);
  std::vector<int> neighbors;
  EXPECT_EQ(grid.RadiusSearch(0U, neighbors, 5U), 1);
}

TEST(UniformGrid2D, HeightPreFilterKeepsTheInclusiveUpperBoundary)
{
  EXPECT_TRUE(ShouldProjectForNeighborSearch(pcl::PointXYZ{1.0F, 2.0F, 1.0F}, 1.0));
  EXPECT_FALSE(ShouldProjectForNeighborSearch(pcl::PointXYZ{1.0F, 2.0F, 1.01F}, 1.0));
}

TEST(UniformGrid2D, MatchesBruteForceForDeterministicRandomCloud)
{
  constexpr double radius = 0.7;
  std::mt19937 generator(20260722U);
  std::uniform_real_distribution<float> coordinate(-20.0F, 20.0F);
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(500U);
  for (std::size_t i = 0U; i < 500U; ++i) {
    cloud->push_back(pcl::PointXYZ{coordinate(generator), coordinate(generator), 0.0F});
  }

  UniformGrid2D grid;
  grid.Build(cloud, radius);
  std::vector<int> actual;
  for (std::size_t query_index = 0U; query_index < cloud->size(); ++query_index) {
    grid.RadiusSearch(query_index, actual, static_cast<unsigned int>(cloud->size()));
    auto expected = BruteForceNeighbors(*cloud, query_index, radius);
    std::sort(actual.begin(), actual.end());
    std::sort(expected.begin(), expected.end());
    EXPECT_EQ(actual, expected) << "query index " << query_index;
  }
}

}  // namespace
}  // namespace residual_ground_filter
