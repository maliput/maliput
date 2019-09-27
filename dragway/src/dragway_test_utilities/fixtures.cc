#include "dragway_test_utilities/fixtures.h"

#include <limits>

#include "dragway/road_geometry.h"

using maliput::api::GeoPosition;
using maliput::api::RoadGeometryId;

namespace maliput {
namespace dragway {

DragwayBasedTest::DragwayBasedTest()
    : dragway_(RoadGeometryId("my_dragway"), kNumLanes, kLength, kLaneWidth, kShoulderWidth, kMaxHeight,
               std::numeric_limits<double>::epsilon() /* linear_tolerance */,
               std::numeric_limits<double>::epsilon() /* angular_tolerance */),
      right_lane_(dragway_.ToRoadPosition(GeoPosition(0, -kLaneWidth, 0)).road_position.lane),
      center_lane_(dragway_.ToRoadPosition(GeoPosition(0, 0, 0)).road_position.lane),
      left_lane_(dragway_.ToRoadPosition(GeoPosition(0, kLaneWidth, 0)).road_position.lane) {}

void DragwayBasedTest::SetUp() {
  ASSERT_TRUE(right_lane_->to_left());
  ASSERT_TRUE(center_lane_->to_left());
  ASSERT_TRUE(center_lane_->to_right());
  ASSERT_TRUE(left_lane_->to_right());
}

}  // namespace dragway
}  // namespace maliput
