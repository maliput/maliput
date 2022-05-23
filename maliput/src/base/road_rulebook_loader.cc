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
#include "maliput/base/road_rulebook_loader.h"

#include <sstream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/rule_registry.h"
#include "maliput/base/yaml_conversion.h"
#include "maliput/common/logger.h"
#include "maliput/common/maliput_throw.h"

using maliput::api::Lane;
using maliput::api::LaneId;
using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::SRange;
using maliput::api::rules::BulbGroup;
using maliput::api::rules::DirectionUsageRule;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::Rule;
using maliput::api::rules::TrafficLight;
using maliput::api::rules::UniqueBulbGroupId;

namespace YAML {

template <>
struct convert<RightOfWayRule::State::YieldGroup> {
  static Node encode(const RightOfWayRule::State::YieldGroup& rhs) {
    Node node;
    for (const auto& rule_id : rhs) {
      node.push_back(rule_id.string());
    }
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  // NOLINTNEXTLINE(runtime/references).
  static bool decode(const Node& node, RightOfWayRule::State::YieldGroup& rhs) {
    if (!node.IsSequence()) {
      return false;
    }
    for (const YAML::Node& yield_node : node) {
      rhs.push_back(RightOfWayRule::Id(yield_node.as<std::string>()));
    }
    return true;
  }
};

template <>
struct convert<DirectionUsageRule::State::Severity> {
  static Node encode(const DirectionUsageRule::State::Severity& rhs) {
    Node node;
    if (rhs == DirectionUsageRule::State::Severity::kPreferred) {
      node = "Preferred";
      return node;
    } else {
      MALIPUT_THROW_UNLESS(rhs == DirectionUsageRule::State::Severity::kStrict);
      node = "Strict";
      return node;
    }
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  static bool decode(const Node& node,
                     // NOLINTNEXTLINE(runtime/references).
                     DirectionUsageRule::State::Severity& rhs) {
    const std::string severity = node.as<std::string>();
    if (severity == "Strict") {
      rhs = DirectionUsageRule::State::Severity::kStrict;
      return true;
    } else if (severity == "Preferred") {
      rhs = DirectionUsageRule::State::Severity::kPreferred;
      return true;
    } else {
      std::stringstream s;
      s << "DirectionUsageRule Severity value: \"" << severity << "\" "
        << " is neither \"Preferred\" or \"Strict\"";
      maliput::log()->debug(s.str());
      return false;
    }
  }
};

template <>
struct convert<DirectionUsageRule::State::Type> {
  static Node encode(const DirectionUsageRule::State::Type& rhs) {
    const auto mapper = DirectionUsageRule::StateTypeMapper();
    Node node;
    node = mapper.at(rhs);
    return node;
  }

  // The following API is required by yaml-cpp. See this web page for more
  // information:
  // https://github.com/jbeder/yaml-cpp/wiki/Tutorial#converting-tofrom-native-data-types
  static bool decode(const Node& node,
                     // NOLINTNEXTLINE(runtime/references).
                     DirectionUsageRule::State::Type& rhs) {
    const std::string type_string = node.as<std::string>();
    const auto mapper = DirectionUsageRule::StateTypeMapper();
    for (const auto& type : mapper) {
      if (type.second == type_string) {
        rhs = type.first;
        return true;
      }
    }

    std::stringstream s;
    bool first{true};
    s << "DirectionUsageRule Type value: \"" << type_string << "\" is invalid. "
      << " Valid Options: [";
    for (const auto& type : mapper) {
      if (!first) {
        s << ", ";
      } else {
        first = false;
      }
      s << type.second;
    }
    s << "]";
    maliput::log()->debug(s.str());
    return false;
  }
};

}  // namespace YAML

namespace maliput {
namespace {

SRange ObtainSRange(const Lane* lane, const YAML::Node& lane_node) {
  if (lane_node["SRange"]) {
    const SRange srange = lane_node["SRange"].as<SRange>();
    MALIPUT_THROW_UNLESS(srange.s0() >= 0);
    MALIPUT_THROW_UNLESS(srange.s1() <= lane->length());
    return srange;
  } else {
    return SRange(0, lane->length());
  }
}

LaneSRange BuildLaneSRange(const api::RoadGeometry* road_geometry, const YAML::Node& lane_node) {
  MALIPUT_THROW_UNLESS(lane_node.IsMap());
  MALIPUT_THROW_UNLESS(lane_node["Lane"].IsDefined());
  const LaneId lane_id(lane_node["Lane"].as<std::string>());
  const Lane* lane = road_geometry->ById().GetLane(lane_id);
  if (lane == nullptr) {
    MALIPUT_THROW_MESSAGE("Trying to generate a LaneSRange for [" + lane_id.string() +
                          "], but the lane couldn't be found within the RoadGeometry.");
  }
  const SRange s_range = ObtainSRange(lane, lane_node);
  return LaneSRange(lane_id, s_range);
}

LaneSRoute BuildLaneSRoute(const api::RoadGeometry* road_geometry, const YAML::Node& zone_node) {
  MALIPUT_THROW_UNLESS(zone_node.IsSequence());
  std::vector<LaneSRange> ranges;
  for (const YAML::Node& lane_node : zone_node) {
    ranges.emplace_back(BuildLaneSRange(road_geometry, lane_node));
  }
  return LaneSRoute(ranges);
}

// RightOfWayRule Loading.
namespace {

std::vector<RightOfWayRule::State> BuildRightOfWayStates(const YAML::Node& states_node) {
  MALIPUT_THROW_UNLESS(states_node.IsMap());
  std::vector<RightOfWayRule::State> states;
  if (states_node["Go"]) {
    states.push_back(RightOfWayRule::State(RightOfWayRule::State::Id("Go"), RightOfWayRule::State::Type::kGo,
                                           states_node["Go"].as<RightOfWayRule::State::YieldGroup>()));
  }
  if (states_node["Stop"]) {
    states.push_back(RightOfWayRule::State(RightOfWayRule::State::Id("Stop"), RightOfWayRule::State::Type::kStop,
                                           states_node["Stop"].as<RightOfWayRule::State::YieldGroup>()));
  }
  if (states_node["StopThenGo"]) {
    states.push_back(RightOfWayRule::State(RightOfWayRule::State::Id("StopThenGo"),
                                           RightOfWayRule::State::Type::kStopThenGo,
                                           states_node["StopThenGo"].as<RightOfWayRule::State::YieldGroup>()));
  }
  if (states.size() != states_node.size()) {
    std::stringstream s;
    s << "RightOfWayRule contained invalid states. It specified the following "
      << "states: [";
    bool first{true};
    for (const auto state : states_node) {
      if (!first) {
        s << ", ";
      }
      s << state.first.as<std::string>();
      first = false;
    }
    s << "]. The valid states are: [Go, Stop, StopThenGo]";
    throw std::domain_error(s.str());
  }
  return states;
}

RightOfWayRule::ZoneType BuildRightOfWayZoneType(const YAML::Node& rule_node) {
  if (rule_node["ZoneType"]) {
    const std::string zone_type = rule_node["ZoneType"].as<std::string>();
    if (zone_type == "StopExcluded") {
      return RightOfWayRule::ZoneType::kStopExcluded;
    } else if (zone_type == "StopAllowed") {
      return RightOfWayRule::ZoneType::kStopAllowed;
    } else {
      std::stringstream s;
      s << "Specified zone type of \"" << zone_type << "\" is neither "
        << "\"StopExcluded\" or \"StopAllowed\".";
      throw std::domain_error(s.str());
    }
  } else {
    return RightOfWayRule::ZoneType::kStopExcluded;
  }
}

RightOfWayRule::RelatedBulbGroups BuildRelatedBulbGroups(const YAML::Node& rule_node) {
  if (rule_node["RelatedBulbGroups"]) {
    MALIPUT_THROW_UNLESS(rule_node["RelatedBulbGroups"].IsMap());

    RightOfWayRule::RelatedBulbGroups related_bulb_groups;

    for (const auto& traffic_light_bulb_group : rule_node["RelatedBulbGroups"]) {
      const TrafficLight::Id traffic_light_id(traffic_light_bulb_group.first.as<std::string>());

      std::vector<BulbGroup::Id> bulb_groups;
      for (auto& bulb_groups_node : traffic_light_bulb_group.second) {
        bulb_groups.emplace_back(bulb_groups_node.as<std::string>());
      }

      MALIPUT_THROW_UNLESS(related_bulb_groups.emplace(traffic_light_id, bulb_groups).second);
    }
    return related_bulb_groups;
  }
  return {};
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
RightOfWayRule BuildRightOfWayRule(const api::RoadGeometry* road_geometry, const YAML::Node& rule_node) {
  MALIPUT_THROW_UNLESS(rule_node.IsMap());
  MALIPUT_THROW_UNLESS(rule_node["ID"].IsDefined());
  const RightOfWayRule::Id rule_id(rule_node["ID"].as<std::string>());

  const YAML::Node& states_node = rule_node["States"];
  MALIPUT_THROW_UNLESS(states_node.IsDefined());
  const std::vector<RightOfWayRule::State> states = BuildRightOfWayStates(states_node);

  const YAML::Node& zone_node = rule_node["Zone"];
  MALIPUT_THROW_UNLESS(zone_node.IsDefined());
  const LaneSRoute zone = BuildLaneSRoute(road_geometry, zone_node);

  return RightOfWayRule(rule_id, zone, BuildRightOfWayZoneType(rule_node), states, BuildRelatedBulbGroups(rule_node));
}

Rule::Id GetRuleIdFrom(const Rule::TypeId& rule_type_id, const RightOfWayRule::Id& right_of_way_rule_id) {
  return maliput::api::rules::Rule::Id(rule_type_id.string() + "/" + right_of_way_rule_id.string());
}
#pragma GCC diagnostic pop

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
DiscreteValueRule BuildVehicleStopInZoneBehaviourDiscreteValueRule(const RightOfWayRule& right_of_way_rule) {
  return DiscreteValueRule(
      GetRuleIdFrom(VehicleStopInZoneBehaviorRuleTypeId(), right_of_way_rule.id()),
      VehicleStopInZoneBehaviorRuleTypeId(), right_of_way_rule.zone(),
      {DiscreteValueRule::DiscreteValue{Rule::State::kStrict, Rule::RelatedRules{}, Rule::RelatedUniqueIds{},
                                        right_of_way_rule.zone_type() == RightOfWayRule::ZoneType::kStopExcluded
                                            ? "DoNotStop"
                                            : "UnconstrainedParking"}});
}

DiscreteValueRule BuildRightOfWayTypeDiscreteValueRule(const RightOfWayRule& right_of_way_rule,
                                                       const DiscreteValueRule::Id& vehicle_stop_in_zone_rule_id) {
  const std::unordered_map<RightOfWayRule::State::Type, std::string> right_of_way_rule_state_types{
      {RightOfWayRule::State::Type::kGo, "Go"},
      {RightOfWayRule::State::Type::kStop, "Stop"},
      {RightOfWayRule::State::Type::kStopThenGo, "StopThenGo"},
  };
#pragma GCC diagnostic pop

  Rule::RelatedUniqueIds related_unique_ids{{RelatedUniqueIdsKeys::kBulbGroup, {}}};
  for (const auto& pair_traffic_light_id_vector_bulb_group_id : right_of_way_rule.related_bulb_groups()) {
    for (const auto& bulb_group_id : pair_traffic_light_id_vector_bulb_group_id.second) {
      related_unique_ids.at(RelatedUniqueIdsKeys::kBulbGroup)
          .push_back(UniqueBulbGroupId{pair_traffic_light_id_vector_bulb_group_id.first, bulb_group_id});
    }
  }
  std::vector<DiscreteValueRule::DiscreteValue> discrete_values;
  for (const auto& state : right_of_way_rule.states()) {
    Rule::RelatedRules related_rules;
    related_rules.emplace(std::pair<std::string, std::vector<Rule::Id>>{VehicleStopInZoneBehaviorRuleTypeId().string(),
                                                                        {vehicle_stop_in_zone_rule_id}});
    std::vector<Rule::Id> rule_ids;
    for (const auto& yield_id : state.second.yield_to()) {
      rule_ids.push_back(GetRuleIdFrom(RightOfWayRuleTypeId(), yield_id));
    }
    related_rules.emplace(std::pair<std::string, std::vector<Rule::Id>>{RelatedRulesKeys::kYieldGroup, rule_ids});
    discrete_values.push_back(DiscreteValueRule::DiscreteValue{Rule::State::kStrict, related_rules, related_unique_ids,
                                                               right_of_way_rule_state_types.at(state.second.type())});
  }

  return DiscreteValueRule(GetRuleIdFrom(RightOfWayRuleTypeId(), right_of_way_rule.id()), RightOfWayRuleTypeId(),
                           right_of_way_rule.zone(), discrete_values);
}

}  // namespace

// DirectionUsageRule loading
namespace {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
std::vector<DirectionUsageRule::State> BuildDirectionUsageStates(const YAML::Node& states_node) {
  MALIPUT_THROW_UNLESS(states_node.IsSequence());
  std::vector<DirectionUsageRule::State> states;
  for (const YAML::Node& state_node : states_node) {
    MALIPUT_THROW_UNLESS(state_node["ID"].IsDefined());
    MALIPUT_THROW_UNLESS(state_node["Severity"].IsDefined());
    MALIPUT_THROW_UNLESS(state_node["Type"].IsDefined());
    states.push_back(DirectionUsageRule::State(DirectionUsageRule::State::Id(state_node["ID"].as<std::string>()),
                                               state_node["Type"].as<DirectionUsageRule::State::Type>(),
                                               state_node["Severity"].as<DirectionUsageRule::State::Severity>()));
  }
  return states;
}

DirectionUsageRule BuildDirectionUsageRule(const api::RoadGeometry* road_geometry, const YAML::Node& rule_node) {
  MALIPUT_THROW_UNLESS(rule_node.IsMap());
  MALIPUT_THROW_UNLESS(rule_node["ID"].IsDefined());
  const DirectionUsageRule::Id rule_id(rule_node["ID"].as<std::string>());

  const YAML::Node& states_node = rule_node["States"];
  MALIPUT_THROW_UNLESS(states_node.IsDefined());
  const std::vector<DirectionUsageRule::State> states = BuildDirectionUsageStates(states_node);

  const YAML::Node& zone_node = rule_node["Zone"];
  MALIPUT_THROW_UNLESS(zone_node.IsDefined());
  const LaneSRange zone = BuildLaneSRange(road_geometry, zone_node);

  return DirectionUsageRule(rule_id, zone, states);
}
#pragma GCC diagnostic pop
}  // namespace

std::unique_ptr<api::rules::RoadRulebook> BuildFrom(const api::RoadGeometry* road_geometry,
                                                    const YAML::Node& root_node) {
  MALIPUT_THROW_UNLESS(root_node.IsMap());
  const YAML::Node& rulebook_node = root_node["RoadRulebook"];
  MALIPUT_THROW_UNLESS(rulebook_node.IsDefined());
  MALIPUT_THROW_UNLESS(rulebook_node.IsMap());
  const YAML::Node& right_of_way_rules_node = rulebook_node["RightOfWayRules"];
  MALIPUT_THROW_UNLESS(right_of_way_rules_node.IsDefined());
  MALIPUT_THROW_UNLESS(right_of_way_rules_node.IsSequence());
  std::unique_ptr<ManualRulebook> rulebook = std::make_unique<ManualRulebook>();
  for (const YAML::Node& right_of_way_rule_node : right_of_way_rules_node) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    const RightOfWayRule right_of_way_rule{BuildRightOfWayRule(road_geometry, right_of_way_rule_node)};
#pragma GCC diagnostic pop
    const DiscreteValueRule vehicle_stop_in_zone_behaviour_discrete_value_rule{
        BuildVehicleStopInZoneBehaviourDiscreteValueRule(right_of_way_rule)};
    rulebook->AddRule(right_of_way_rule);
    rulebook->AddRule(vehicle_stop_in_zone_behaviour_discrete_value_rule);
    rulebook->AddRule(BuildRightOfWayTypeDiscreteValueRule(right_of_way_rule,
                                                           vehicle_stop_in_zone_behaviour_discrete_value_rule.id()));
  }
  const YAML::Node& direction_usage_rules_node = rulebook_node["DirectionUsageRules"];
  MALIPUT_THROW_UNLESS(direction_usage_rules_node.IsDefined());
  MALIPUT_THROW_UNLESS(direction_usage_rules_node.IsSequence());
  for (const YAML::Node& direction_usage_rule_node : direction_usage_rules_node) {
    rulebook->AddRule(BuildDirectionUsageRule(road_geometry, direction_usage_rule_node));
  }
  // TODO(liang.fok) Add loading of speed limit rules.
  return rulebook;
}
}  // namespace

std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebook(const api::RoadGeometry* road_geometry,
                                                           const std::string& input) {
  return BuildFrom(road_geometry, YAML::Load(input));
}

std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebookFromFile(const api::RoadGeometry* road_geometry,
                                                                   const std::string& filename) {
  return BuildFrom(road_geometry, YAML::LoadFile(filename));
}

}  // namespace maliput
