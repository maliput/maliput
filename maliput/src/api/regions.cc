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

bool SRange::Intersects(const SRange& s_range, double tolerance) const {
  MALIPUT_THROW_UNLESS(std::min(s0(), s1()) >= 0 && std::min(s_range.s0(), s_range.s1()) >= 0);
  if (tolerance < 0.) {
    // When it is negative, tolerance's absolute value can not be bigger than half size of minor SRange.
    MALIPUT_THROW_UNLESS(std::min(size(), s_range.size()) / 2. >= std::fabs(tolerance));
  }
  const SRange wider_s_range(std::min(s0(), s1()) - tolerance, std::max(s0(), s1()) + tolerance);
  return !((std::max(s_range.s0(), s_range.s1()) < wider_s_range.s0()) ||
           (std::min(s_range.s0(), s_range.s1()) > wider_s_range.s1()));
}

drake::optional<SRange> SRange::GetIntersection(const SRange& s_range, double tolerance) const {
  if (Intersects(s_range, tolerance)) {
    const SRange wider_s_range(std::min(s0(), s1()) - tolerance, std::max(s0(), s1()) + tolerance);
    const double max = std::max(s_range.s0(), s_range.s1()) >= wider_s_range.s1()
                           ? std::max(s0(), s1())
                           : std::max(s_range.s0(), s_range.s1());
    const double min = std::min(s_range.s0(), s_range.s1()) <= wider_s_range.s0()
                           ? std::min(s0(), s1())
                           : std::min(s_range.s0(), s_range.s1());

    return drake::optional<SRange>{SRange(min, max)};
  }
  return drake::nullopt;
}

bool LaneSRange::Intersects(const LaneSRange& lane_s_range, const double tolerance) const {
  return lane_id_ == lane_s_range.lane_id() ? s_range_.Intersects(lane_s_range.s_range(), tolerance) : false;
}

bool LaneSRoute::Intersects(const LaneSRoute& lane_s_route, double tolerance) const {
  for (const auto& s_range : ranges()) {
    const auto lane_s_range_it =
        std::find_if(lane_s_route.ranges().begin(), lane_s_route.ranges().end(),
                     [s_range](const LaneSRange& lane_s_range) { return s_range.lane_id() == lane_s_range.lane_id(); });
    if (lane_s_range_it != lane_s_route.ranges().end()) {
      if (s_range.Intersects(*lane_s_range_it, tolerance)) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace api
}  // namespace maliput
