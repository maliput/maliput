#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_geometry.h"
#include "maliput/common/assertion_error.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/maliput_types_compare.h"
#include "multilane/builder.h"
#include "multilane/loader.h"

namespace maliput {
namespace {

using api::GeoPosition;
using api::LaneId;
using api::LanePosition;
using api::LaneSRange;
using api::LaneSRoute;
using api::RoadGeometry;

GTEST_TEST(WaypointsTest, Waypoints) {
  const std::string kId = "long_start_and_end_lanes";
  const std::string kComputationPolicy = "prefer-accuracy";
  const double kScaleLength = 1.0;
  const double kLaneWidth = 6.0;
  const double kLeftShoulder = 5.0;
  const double kRightShoulder = 5.0;
  const api::HBounds kElevationBounds{0., 5.};
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.5;
  const std::string kMultilaneYaml = fmt::format(
      R"R(maliput_multilane_builder:
  id: {}
  computation_policy: {}
  scale_length: {}
  lane_width: {}
  left_shoulder: {}
  right_shoulder: {}
  elevation_bounds: [{}, {}]
  linear_tolerance: {}
  angular_tolerance: {}
  points:
    start:
      xypoint: [0, 0, 0]
      zpoint: [0, 0, 0, 0]
  connections:
    0:
      lanes: [1, 0, 0]  # num_lanes, ref_lane, r_ref
      start: ["ref", "points.start.forward"]
      length: 5
      z_end: ["ref", [0, 0, 0]]
    1:
      lanes: [1, 0, 0]
      start: ["ref", "connections.0.end.ref.forward"]
      length: 10
      z_end: ["ref", [0, 0, 0]]
)R",
      kId, kComputationPolicy, kScaleLength, kLaneWidth, kLeftShoulder, kRightShoulder, kElevationBounds.min(),
      kElevationBounds.max(), kLinearTolerance, kAngularTolerance);

  std::unique_ptr<const RoadGeometry> rg = Load(multilane::BuilderFactory(), kMultilaneYaml);
  ASSERT_NE(rg, nullptr);
  const maliput::api::Lane* lane_one = rg->junction(0)->segment(0)->lane(0);
  const maliput::api::Lane* lane_two = rg->junction(1)->segment(0)->lane(0);

  const LaneSRange range_one{lane_one->id(), {1., 5.}};
  const LaneSRange range_two{lane_two->id(), {0., 10.}};

  const LaneSRoute route{std::vector<LaneSRange>{range_one, range_two}};
  EXPECT_EQ(route.length(), 14.0);
  const double kSampleSStep{4.0};
  std::vector<GeoPosition> waypoints = rg->SampleAheadWaypoints(route, kSampleSStep);

  ASSERT_EQ(waypoints.size(), 5);
  EXPECT_TRUE(
      api::test::IsGeoPositionClose(GeoPosition(waypoints[1].x() + kSampleSStep, waypoints[1].y(), waypoints[1].z()),
                                    waypoints[2], kLinearTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      GeoPosition(waypoints[0].x() + 14, waypoints[0].y(), waypoints[0].z()), waypoints[4],
      kLinearTolerance));
  EXPECT_TRUE(api::test::IsGeoPositionClose(GeoPosition(waypoints[3].x() + 2.0, waypoints[0].y(), waypoints[0].z()),
                                            waypoints[4], kLinearTolerance));

  const LaneSRange shorter_than_sample_step_range_one{lane_one->id(), {5., 5.}};
  const LaneSRange shorter_than_sample_step_range_two{lane_two->id(), {0., 2.}};
  std::vector<LaneSRange> shorter_ranges;
  shorter_ranges.emplace_back(shorter_than_sample_step_range_one);
  shorter_ranges.emplace_back(shorter_than_sample_step_range_two);
  const LaneSRoute shorter_route(shorter_ranges);

  EXPECT_EQ(shorter_route.length(), 2.0);
  const double kSampleSStepBiggerThanTotalRouteLength = 15.0;
  waypoints = rg->SampleAheadWaypoints(shorter_route, kSampleSStepBiggerThanTotalRouteLength);
  ASSERT_EQ(waypoints.size(), 2);
  EXPECT_TRUE(api::test::IsGeoPositionClose(
      lane_two->ToGeoPosition(LanePosition(shorter_than_sample_step_range_two.s_range().s1(), 0.0, 0.0)), waypoints[1],
      kLinearTolerance));

  const LaneSRange non_existent_range_lane{LaneId("non-existent"), {0., 2.0}};
  const LaneSRoute non_existent_route{std::vector<LaneSRange>{non_existent_range_lane}};
  EXPECT_THROW(rg->SampleAheadWaypoints(non_existent_route, kSampleSStep), maliput::common::assertion_error);

  const double kNegativeStep = -1.0;
  EXPECT_THROW(rg->SampleAheadWaypoints(route, kNegativeStep), maliput::common::assertion_error);

  const double step_smaller_than_linear_tolerance = kLinearTolerance / 2.0;
  const LaneSRange linear_tolerance_range{lane_one->id(), {0., kLinearTolerance}};
  const LaneSRoute route_with_linear_tolerance_lane{std::vector<LaneSRange>{linear_tolerance_range}};
  waypoints = rg->SampleAheadWaypoints(route_with_linear_tolerance_lane, step_smaller_than_linear_tolerance);
  EXPECT_EQ(waypoints.size(), 2);
};

}  // namespace
}  // namespace maliput
