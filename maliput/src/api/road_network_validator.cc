// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
void CheckLaneSRouteContiguity(const RoadGeometry* road_geometry, const LaneSRoute& lane_s_route) {
  // Iterating through the lanes of a rule.
  for (int i = 0; i < static_cast<int>(lane_s_route.ranges().size()) - 1; ++i) {
    const LaneSRange lane_range_a = lane_s_route.ranges()[i];
    const LaneSRange lane_range_b = lane_s_route.ranges()[i + 1];
    if (!IsContiguous(lane_range_a, lane_range_b, road_geometry)) {
      MALIPUT_THROW_MESSAGE("LaneSRange(id: " + lane_range_a.lane_id().string() +
                            ", s0:  " + std::to_string(lane_range_a.s_range().s0()) +
                            ", s1: " + std::to_string(lane_range_a.s_range().s1()) +
                            ") is not G1 contiguous with "
                            "LaneSRange(id: " +
                            lane_range_b.lane_id().string() + ", s0:  " + std::to_string(lane_range_b.s_range().s0()) +
                            ", s1: " + std::to_string(lane_range_b.s_range().s1()) + ").");
    }
  }
}

// Evaluates G1 contiguity for all rule zones.
// @throws common::assertion_error When any rule zone is not G1 contiguous.
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
// have a BranchPoint at each endpoint. Also, IdIndex must contain references to
// all entities within the RoadGeometry graph.
void CheckRoadGeometryHierarchyConsistency(const RoadNetwork& road_network) {
  const RoadGeometry* rg = road_network.road_geometry();
  MALIPUT_THROW_UNLESS(rg != nullptr);

  const RoadGeometry::IdIndex& id_index = rg->ById();
  MALIPUT_THROW_UNLESS(rg->num_junctions() > 0);
  for (int i = 0; i < rg->num_junctions(); ++i) {
    const Junction* junction = rg->junction(i);
    MALIPUT_THROW_UNLESS(junction != nullptr);
    MALIPUT_THROW_UNLESS(junction->road_geometry() == rg);
    MALIPUT_THROW_UNLESS(junction == id_index.GetJunction(junction->id()));
    MALIPUT_THROW_UNLESS(junction->num_segments() > 0);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const Segment* segment = junction->segment(j);
      MALIPUT_THROW_UNLESS(segment != nullptr);
      MALIPUT_THROW_UNLESS(segment->junction() == junction);
      MALIPUT_THROW_UNLESS(segment == id_index.GetSegment(segment->id()));
      MALIPUT_THROW_UNLESS(segment->num_lanes() > 0);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const Lane* lane = segment->lane(k);
        MALIPUT_THROW_UNLESS(lane != nullptr);
        MALIPUT_THROW_UNLESS(lane->segment() == segment);
        MALIPUT_THROW_UNLESS(lane == id_index.GetLane(lane->id()));
        const BranchPoint* bp_start = lane->GetBranchPoint(LaneEnd::Which::kStart);
        MALIPUT_THROW_UNLESS(bp_start != nullptr);
        const BranchPoint* bp_finish = lane->GetBranchPoint(LaneEnd::Which::kFinish);
        MALIPUT_THROW_UNLESS(bp_finish != nullptr);
      }
    }
  }

  MALIPUT_THROW_UNLESS(rg->num_branch_points() >= 2);
  for (int i = 0; i < rg->num_branch_points(); ++i) {
    const BranchPoint* bp = rg->branch_point(i);
    MALIPUT_THROW_UNLESS(bp != nullptr);
    MALIPUT_THROW_UNLESS(bp->road_geometry() == rg);
    MALIPUT_THROW_UNLESS(bp == id_index.GetBranchPoint(bp->id()));
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
    const TrafficLight* traffic_light = road_network.traffic_light_book()->GetTrafficLight(traffic_light_id);
    MALIPUT_THROW_UNLESS(traffic_light != nullptr);
    MALIPUT_THROW_UNLESS(traffic_light->GetBulbGroup(bulb_group_id) != nullptr);
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

// Walks through all rules::Phases in rules::PhaseRingBook and calls
// `evaluate_phase`.
//
// `evaluate_phase` is a functor that throws if rules::Phase is invalid.
//
// @throws common::assertion_error When querying for a rules::PhaseRing to
// `road_network.phase_ring_book()` returns nullopt.
void WalkPhases(const RoadNetwork& road_network, std::function<void(const rules::Phase&)> evaluate_phase) {
  const rules::PhaseRingBook* phase_ring_book = road_network.phase_ring_book();
  MALIPUT_THROW_UNLESS(phase_ring_book != nullptr);
  const std::vector<rules::PhaseRing::Id> phase_ring_ids = phase_ring_book->GetPhaseRings();
  for (const rules::PhaseRing::Id& phase_ring_id : phase_ring_ids) {
    const std::optional<rules::PhaseRing> phase_ring = phase_ring_book->GetPhaseRing(phase_ring_id);
    MALIPUT_THROW_UNLESS(phase_ring.has_value());
    for (const auto& phase_id_phase : phase_ring->phases()) {
      evaluate_phase(phase_id_phase.second);
    }
  }
}

// Evaluates that every rules::DiscreteValueRuleStates contained in
// rules::Phases reference rules::Rules in `road_network.rulebook()` and their
// values.
void CheckPhaseDiscreteValueRuleStates(const RoadNetwork& road_network) {
  auto evaluate_phase = [rulebook = road_network.rulebook()](const rules::Phase& phase) {
    for (const auto& rule_id_value : phase.discrete_value_rule_states()) {
      const rules::DiscreteValueRule rule = rulebook->GetDiscreteValueRule(rule_id_value.first);
      if (std::find(rule.states().begin(), rule.states().end(), rule_id_value.second) == rule.states().end()) {
        MALIPUT_THROW_MESSAGE("DiscreteValueRuleStates have an unknown DiscreteValue referenced by Rule(id: " +
                              rule.id().string() + ") in Phase(id: " + phase.id().string() + ")");
      }
    }
  };
  WalkPhases(road_network, evaluate_phase);
}

// Evaluates that every rules::BulbStates contained in rules::Phases reference
// rules::Bulbs in `road_network.traffic_light_book()`.
void CheckPhasesBulbStates(const RoadNetwork& road_network) {
  auto evaluate_phase = [traffic_light_book = road_network.traffic_light_book()](const rules::Phase& phase) {
    if (!phase.bulb_states().has_value()) {
      return;
    }
    for (const auto& unique_bulb_id_state : *phase.bulb_states()) {
      const rules::TrafficLight* traffic_light =
          traffic_light_book->GetTrafficLight(unique_bulb_id_state.first.traffic_light_id());
      if (traffic_light == nullptr) {
        MALIPUT_THROW_MESSAGE("TrafficLight(id: " + unique_bulb_id_state.first.traffic_light_id().string() +
                              "), which is referenced by Phase(id: " + phase.id().string() +
                              ") does not exist in TrafficLightBook.");
      }
      const rules::BulbGroup* bulb_group = traffic_light->GetBulbGroup(unique_bulb_id_state.first.bulb_group_id());
      if (bulb_group == nullptr) {
        MALIPUT_THROW_MESSAGE("BulbGroup(id: " + unique_bulb_id_state.first.bulb_group_id().string() +
                              "), which is referenced by Phase(id: " + phase.id().string() +
                              ") does not exist in TrafficLightBook.");
      }
      const rules::Bulb* bulb = bulb_group->GetBulb(unique_bulb_id_state.first.bulb_id());
      if (bulb == nullptr) {
        MALIPUT_THROW_MESSAGE("Bulb(id: " + unique_bulb_id_state.first.bulb_id().string() +
                              "), which is referenced by Phase(id: " + phase.id().string() +
                              ") does not exist in TrafficLightBook.");
      }
      if (std::find(bulb->states().begin(), bulb->states().end(), unique_bulb_id_state.second) ==
          bulb->states().end()) {
        MALIPUT_THROW_MESSAGE("BulbStates have an unknown BulbState referenced by UniqueBulbId(id: " +
                              unique_bulb_id_state.first.string() + ") in Phase(id: " + phase.id().string() + ")");
      }
    }
  };
  WalkPhases(road_network, evaluate_phase);
}

// Evaluates if `rule_id` is in `discrete_value_rules` or `range_value_rules`.
bool IsRuleIdIn(const rules::Rule::Id rule_id,
                const std::map<rules::DiscreteValueRule::Id, rules::DiscreteValueRule> discrete_value_rules,
                const std::map<rules::RangeValueRule::Id, rules::RangeValueRule> range_value_rules) {
  return (discrete_value_rules.find(rule_id) != discrete_value_rules.end()) ||
         (range_value_rules.find(rule_id) != range_value_rules.end());
}

// Evaluates if rules::DiscreteValueRules and rules::RangeValueRules contain
// states whose Rule::RelatedRules point to existent rule::Rules in
// `road_network.rulebook()`.
void CheckRelatedRues(const RoadNetwork& road_network) {
  const rules::RoadRulebook::QueryResults rules = road_network.rulebook()->Rules();

  for (const auto& kv : rules.discrete_value_rules) {
    for (const auto& value : kv.second.states()) {
      for (const auto& group_related_rule_ids : value.related_rules) {
        for (const auto& related_rule_id : group_related_rule_ids.second) {
          if (!IsRuleIdIn(related_rule_id, rules.discrete_value_rules, rules.range_value_rules)) {
            MALIPUT_THROW_MESSAGE("DiscreteValueRule(id:" + kv.first.string() +
                                  ") has a DiscreteValue with a "
                                  "RelatedRule pointing to id:" +
                                  related_rule_id.string() +
                                  " and it does not exist in the "
                                  "RoadRulebook.");
          }
        }
      }
    }
  }

  for (const auto& kv : rules.range_value_rules) {
    for (const auto& range : kv.second.states()) {
      for (const auto& group_related_rule_ids : range.related_rules) {
        for (const auto& related_rule_id : group_related_rule_ids.second) {
          if (!IsRuleIdIn(related_rule_id, rules.discrete_value_rules, rules.range_value_rules)) {
            MALIPUT_THROW_MESSAGE("RangeValueRule(id:" + kv.first.string() +
                                  ") has a Range with a related rule "
                                  "pointing to id:" +
                                  related_rule_id.string() + " and it does not exist in the RoadRulebook.");
          }
        }
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
  if (options.check_phase_discrete_value_rule_states) {
    CheckPhaseDiscreteValueRuleStates(road_network);
  }
  if (options.check_phase_bulb_states) {
    CheckPhasesBulbStates(road_network);
  }
  if (options.check_related_rules) {
    CheckRelatedRues(road_network);
  }
}

}  // namespace api
}  // namespace maliput
