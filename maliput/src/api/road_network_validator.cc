#include "maliput/api/road_network_validator.h"

#include <string>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace {

using rules::BulbGroup;
using rules::TrafficLight;

// Given a `LaneSRoute` this method checks the G1 contiguity
// between of all its `LaneSRange`.
void CheckLaneSRouteContiguity(const RoadGeometry* road_geometry, const LaneSRoute& rule_route) {
  for (int i = 0; i < rule_route.ranges().size() - 1; ++i) {  // Iterating through the lanes of a rule.
    const LaneSRange lane_range_a = rule_route.ranges()[i];
    const LaneSRange lane_range_b = rule_route.ranges()[i + 1];
    if (!IsContiguous(lane_range_a, lane_range_b, road_geometry)) {
      MALIPUT_THROW_MESSAGE("Lanes are not contiguous");
    }
  }
}

// Checks the contiguity between lanes, for both
// linear and angular tolerance.
// @throws common::assertion_error When any of the RoadGeometry's tolerances is not met.
void CheckContiguityBetweenLanes(const RoadNetwork& road_network) {
  const rules::RoadRulebook::QueryResults rules = road_network.rulebook()->Rules();
  const RoadGeometry* const road_geometry = road_network.road_geometry();
  for (const std::pair<rules::DiscreteValueRule::Id, rules::DiscreteValueRule>& key_value :
       rules.discrete_value_rules) {
    CheckLaneSRouteContiguity(road_geometry, key_value.second.zone());
  }
  for (const std::pair<rules::RangeValueRule::Id, rules::RangeValueRule>& key_value : rules.range_value_rules) {
    CheckLaneSRouteContiguity(road_geometry, key_value.second.zone());
  }
}

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

// Checks that TrafficLight::Ids and BulbGroup::Ids in RightOfWayRules
// as RelatedBulbGroups have a supporting entity in TrafficLightBook.
void CheckRelatedBulbGroups(const RoadNetwork& road_network) {
  auto evaluate_related_bulb_group_existance = [&](const TrafficLight::Id& traffic_light_id,
                                                   const BulbGroup::Id& bulb_group_id) {
    const drake::optional<TrafficLight> traffic_light =
        road_network.traffic_light_book()->GetTrafficLight(traffic_light_id);
    MALIPUT_THROW_UNLESS(traffic_light.has_value());
    MALIPUT_THROW_UNLESS(traffic_light->GetBulbGroup(bulb_group_id).has_value());
  };

  const rules::RoadRulebook::QueryResults result = road_network.rulebook()->Rules();
  for (const auto& rule_id_to_rule : result.right_of_way) {
    for (const auto& traffic_light_bulb_groups : rule_id_to_rule.second.related_bulb_groups()) {
      for (const BulbGroup::Id& bulb_group_id : traffic_light_bulb_groups.second) {
        evaluate_related_bulb_group_existance(traffic_light_bulb_groups.first, bulb_group_id);
      }
    }
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
  if (options.check_related_bulb_groups) {
    CheckRelatedBulbGroups(road_network);
  }
  if (options.check_contiguity_rule_zones) {
    CheckContiguityBetweenLanes(road_network);
  }
}

}  // namespace api
}  // namespace maliput
