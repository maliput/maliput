#include "maliput/api/lane.h"
#include "maliput/api/junction.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace api {

bool IsWithinRange(double x, double min, double max, double tolerance) {
  return ((min - tolerance) <= x) && (x <= (max + tolerance));
}

bool Lane::Contains(const LanePosition& lane_position) const {
  const double s = lane_position.s();
  const double r = lane_position.r();
  const double h = lane_position.h();

  const RBounds segment_bounds = this->segment_bounds(s);
  const HBounds elevation_bounds = this->elevation_bounds(s, r);
  const double lane_length = this->length();
  const double linear_tolerance = this->segment()->junction()->road_geometry()->linear_tolerance();

  return IsWithinRange(s, 0., lane_length, linear_tolerance) &&
         IsWithinRange(r, segment_bounds.min(), segment_bounds.max(), linear_tolerance) &&
         IsWithinRange(h, elevation_bounds.min(), elevation_bounds.max(), linear_tolerance);
}

}  // namespace api
}  // namespace maliput
