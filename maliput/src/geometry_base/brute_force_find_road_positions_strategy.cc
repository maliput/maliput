#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"

#include "drake/common/drake_throw.h"

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace geometry_base {

std::vector<maliput::api::RoadPositionResult>
BruteForceFindRoadPositionsStrategy::operator()(
    const maliput::api::RoadGeometry* rg,
    const maliput::api::GeoPosition& geo_position,
    double radius) const {
  DRAKE_THROW_UNLESS(rg != nullptr);
  DRAKE_THROW_UNLESS(radius >= 0.);

  std::vector<maliput::api::RoadPositionResult> road_position_results;

  for (int i = 0; i < rg->num_junctions(); ++i) {
    const maliput::api::Junction* junction = rg->junction(i);
    DRAKE_THROW_UNLESS(junction != nullptr);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const maliput::api::Segment* segment = junction->segment(j);
      DRAKE_THROW_UNLESS(segment != nullptr);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const api::Lane* lane = segment->lane(k);
        DRAKE_THROW_UNLESS(lane != nullptr);
        double distance{};
        maliput::api::GeoPosition nearest_position;
        const maliput::api::LanePosition lane_position =
            lane->ToLanePosition(geo_position, &nearest_position, &distance);

        if (distance <= radius) {
          road_position_results.push_back({
              api::RoadPosition(lane, lane_position),
              nearest_position,
              distance});
        }
      }
    }
  }

  return road_position_results;
}

}  // namespace geometry_base
}  // namespace maliput
