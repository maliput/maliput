#include "maliput/api/regions.h"
#include "maliput/api/road_geometry.h"

namespace maliput {
namespace api {

bool IsContiguous(const LaneSRange& lane_range_a, const LaneSRange& lane_range_b, const RoadGeometry* road_geometry) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  const Lane* lane_a = road_geometry->ById().GetLane(lane_range_a.lane_id());
  MALIPUT_THROW_UNLESS(lane_a != nullptr);
  const Lane* lane_b = road_geometry->ById().GetLane(lane_range_b.lane_id());
  MALIPUT_THROW_UNLESS(lane_b != nullptr);
  // Evaluates G1 contiguity at the end and start of `lane_a` and `lane_b` respectively.
  const GeoPosition geo_position_a = lane_a->ToGeoPosition(LanePosition(lane_range_a.s_range().s1(), 0, 0));
  const GeoPosition geo_position_b = lane_b->ToGeoPosition(LanePosition(lane_range_b.s_range().s0(), 0, 0));
  const Rotation lane_a_rot = lane_a->GetOrientation(LanePosition(lane_range_a.s_range().s1(), 0, 0));
  const Rotation lane_b_rot = lane_b->GetOrientation(LanePosition(lane_range_b.s_range().s0(), 0, 0));
  return geo_position_a.Distance(geo_position_b) < road_geometry->linear_tolerance() &&
         lane_a_rot.Distance(lane_b_rot) < road_geometry->angular_tolerance();
}

}  // namespace api
}  // namespace maliput
