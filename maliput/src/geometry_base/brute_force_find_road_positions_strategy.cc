#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"

#include <limits>

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace geometry_base {

std::vector<maliput::api::RoadPositionResult> BruteForceFindRoadPositionsStrategy(
    const maliput::api::RoadGeometry* rg, const maliput::api::GeoPosition& geo_position, double radius) {
  MALIPUT_THROW_UNLESS(rg != nullptr);
  MALIPUT_THROW_UNLESS(radius >= 0.);

  std::vector<maliput::api::RoadPositionResult> road_position_results;

  for (int i = 0; i < rg->num_junctions(); ++i) {
    const maliput::api::Junction* junction = rg->junction(i);
    MALIPUT_THROW_UNLESS(junction != nullptr);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const maliput::api::Segment* segment = junction->segment(j);
      MALIPUT_THROW_UNLESS(segment != nullptr);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const api::Lane* lane = segment->lane(k);
        MALIPUT_THROW_UNLESS(lane != nullptr);
        double distance{};
        maliput::api::GeoPosition nearest_position;
        const maliput::api::LanePositionResult result = lane->ToLanePosition(geo_position);
        if (radius == std::numeric_limits<double>::infinity() || result.distance <= radius) {
          road_position_results.push_back(
              {api::RoadPosition(lane, result.lane_position), result.nearest_position, result.distance});
        }
      }
    }
  }

  return road_position_results;
}

}  // namespace geometry_base
}  // namespace maliput
