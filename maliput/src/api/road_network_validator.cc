#include "maliput/api/road_network_validator.h"

#include <string>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/road_geometry.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace {

// Confirms full DirectionUsageRule coverage. This is determined by
// verifying that each Lane within the RoadGeometry has an associated
// DirectionUsageRule. In the future, this check could be made even more
// rigorous by confirming that the union of all DirectionUsageRule zones
// covers the whole RoadGeometry.
void CheckDirectionUsageRuleCoverage(const RoadNetwork& road_network) {
  const auto& lanes_map = road_network.road_geometry()->ById().GetLanes();
  for (const auto& lane_map : lanes_map) {
    const LaneId lane_id = lane_map.first;
    const auto results = road_network.rulebook()->FindRules({{lane_id, {0.0, lane_map.second->length()}}}, 0);
    MALIPUT_THROW_UNLESS(results.direction_usage.size() > 0);
  }
}

// Calls RoadGeometry::CheckInvariants() and throws when there are violations.
void ThrowIfThereAreRoadGeometryInvariants(const RoadNetwork& road_network) {
  const std::vector<std::string> violations = road_network.road_geometry()->CheckInvariants();
  MALIPUT_THROW_UNLESS(violations.size() == 0);
}

// Evaluates that Junctions, Segments and BranchPoints are not empty, Lanes
// have a BranchPoint at each endpoint.
void CheckRoadGeometryHierarchyConsistency(const RoadNetwork& road_network) {
  MALIPUT_THROW_UNLESS(road_network.road_geometry()->num_junctions() > 0);
  for (int i = 0; i < road_network.road_geometry()->num_junctions(); ++i) {
    const Junction* junction = road_network.road_geometry()->junction(i);
    MALIPUT_THROW_UNLESS(junction != nullptr);
    MALIPUT_THROW_UNLESS(junction->road_geometry() == road_network.road_geometry());
    MALIPUT_THROW_UNLESS(junction->num_segments() > 0);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const Segment* segment = junction->segment(j);
      MALIPUT_THROW_UNLESS(segment != nullptr);
      MALIPUT_THROW_UNLESS(segment->junction() == junction);
      MALIPUT_THROW_UNLESS(segment->num_lanes() > 0);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const Lane* lane = segment->lane(k);
        MALIPUT_THROW_UNLESS(lane != nullptr);
        MALIPUT_THROW_UNLESS(lane->segment() == segment);
        const BranchPoint* bp_start = lane->GetBranchPoint(LaneEnd::Which::kStart);
        MALIPUT_THROW_UNLESS(bp_start != nullptr);
        const BranchPoint* bp_finish = lane->GetBranchPoint(LaneEnd::Which::kFinish);
        MALIPUT_THROW_UNLESS(bp_finish != nullptr);
      }
    }
  }

  MALIPUT_THROW_UNLESS(road_network.road_geometry()->num_branch_points() >= 2);
  for (int i = 0; i < road_network.road_geometry()->num_branch_points(); ++i) {
    const BranchPoint* bp = road_network.road_geometry()->branch_point(i);
    MALIPUT_THROW_UNLESS(bp != nullptr);
    MALIPUT_THROW_UNLESS(bp->road_geometry() == road_network.road_geometry());
    MALIPUT_THROW_UNLESS(bp->GetASide() != nullptr);
    MALIPUT_THROW_UNLESS(bp->GetBSide() != nullptr);
    MALIPUT_THROW_UNLESS(bp->GetASide()->size() != 0 || bp->GetBSide()->size() != 0);
  }
}

}  // namespace

void ValidateRoadNetwork(const RoadNetwork& road_network, const RoadNetworkValidatorOptions& options) {
  if (options.check_direction_usage_rule_coverage) {
    CheckDirectionUsageRuleCoverage(road_network);
  }
  if (options.check_road_geometry_invariants) {
    ThrowIfThereAreRoadGeometryInvariants(road_network);
  }
  if (options.check_road_geometry_hierarchy) {
    CheckRoadGeometryHierarchyConsistency(road_network);
  }
}

}  // namespace api
}  // namespace maliput
