#include "maliput/geometry_base/filter_positions.h"

#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/test_utilities/maliput_types_compare.h"

namespace maliput {
namespace geometry_base {
namespace {

GTEST_TEST(FilterRoadPositionResultsTest, FilterRoadPositionResults) {
  const double kZeroTolerance{0.};

  const api::Lane* kMockLane1{reinterpret_cast<const api::Lane*>(0xDeadBeef)};
  const api::Lane* kMockLane2{reinterpret_cast<const api::Lane*>(0xDeadC0de)};

  const std::vector<api::RoadPositionResult> unfiltered_positions{
      {api::RoadPosition{kMockLane1, api::LanePosition{1., 2., 3}}, api::GeoPosition{4., 5., 6.}, 0.},
      {api::RoadPosition{kMockLane2, api::LanePosition{7., 8., 9}}, api::GeoPosition{1., 2., 3.}, 1.},
  };

  const auto filter_always_true = [](const api::RoadPositionResult&) { return true; };
  const auto filter_always_false = [](const api::RoadPositionResult&) { return false; };
  const auto filter_lane_1 = [lane_ptr = kMockLane1](const api::RoadPositionResult& road_position_result) {
    return road_position_result.road_position.lane != lane_ptr;
  };

  std::vector<api::RoadPositionResult> result = FilterRoadPositionResults(unfiltered_positions, filter_always_false);
  EXPECT_TRUE(result.empty());

  result = FilterRoadPositionResults(unfiltered_positions, filter_always_true);
  EXPECT_EQ(static_cast<int>(result.size()), 2);
  EXPECT_EQ(result[0].road_position.lane, unfiltered_positions[0].road_position.lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(result[0].road_position.pos, unfiltered_positions[0].road_position.pos,
                                             kZeroTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(result[0].nearest_position, unfiltered_positions[0].nearest_position,
                                            kZeroTolerance));
  EXPECT_NEAR(result[0].distance, unfiltered_positions[0].distance, kZeroTolerance);
  EXPECT_EQ(result[1].road_position.lane, unfiltered_positions[1].road_position.lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(result[1].road_position.pos, unfiltered_positions[1].road_position.pos,
                                             kZeroTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(result[1].nearest_position, unfiltered_positions[1].nearest_position,
                                            kZeroTolerance));
  EXPECT_NEAR(result[1].distance, unfiltered_positions[1].distance, kZeroTolerance);

  result = FilterRoadPositionResults(unfiltered_positions, filter_lane_1);
  EXPECT_EQ(static_cast<int>(result.size()), 1);
  EXPECT_EQ(result[0].road_position.lane, unfiltered_positions[1].road_position.lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(result[0].road_position.pos, unfiltered_positions[1].road_position.pos,
                                             kZeroTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(result[0].nearest_position, unfiltered_positions[1].nearest_position,
                                            kZeroTolerance));
  EXPECT_NEAR(result[0].distance, unfiltered_positions[1].distance, kZeroTolerance);
}

}  // namespace
}  // namespace geometry_base
}  // namespace maliput
