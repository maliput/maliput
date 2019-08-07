#include "maliput/api/road_network_validator.h"

#include "maliput/api/lane.h"

#include "drake/common/drake_throw.h"

namespace maliput {
namespace api {

void ValidateRoadNetwork(const RoadNetwork& road_network, const RoadNetworkValidatorOptions& options) {
  if (options.check_direction_usage_rule_coverage) {
    // Confirms full DirectionUsageRule coverage. This is determined by
    // verifying that each Lane within the RoadGeometry has an associated
    // DirectionUsageRule. In the future, this check could be made even more
    // rigorous by confirming that the union of all DirectionUsageRule zones
    // covers the whole RoadGeometry.
    const auto& lanes_map = road_network.road_geometry()->ById().GetLanes();
    for (const auto& lane_map : lanes_map) {
      const LaneId lane_id = lane_map.first;
      const auto results = road_network.rulebook()->FindRules({{lane_id, {0.0, lane_map.second->length()}}}, 0);
      MALIPUT_THROW_UNLESS(results.direction_usage.size() > 0);
    }
  }
}

}  // namespace api
}  // namespace maliput
