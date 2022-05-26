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
#include <sstream>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/road_rulebook_loader.h"
#include "maliput/base/rule_registry.h"
#include "maliput/base/yaml_conversion.h"
#include "maliput/common/logger.h"
#include "maliput/common/maliput_throw.h"

using maliput::api::Lane;
using maliput::api::LaneId;
using maliput::api::LaneSRange;
using maliput::api::LaneSRoute;
using maliput::api::SRange;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::RangeValueRule;
using maliput::api::rules::Rule;
using maliput::api::rules::RuleRegistry;

namespace maliput {
namespace {

bool HasBaseRuleProperties(const YAML::Node& rule_node) {
  return rule_node[RuleConstants::kId].IsDefined() && rule_node[RuleConstants::kType].IsDefined() &&
         rule_node[RuleConstants::kZone].IsDefined();
}

// Determines whether the `rule_node` corresponds to a api::rules::DiscreteValueRule description.
bool IsDiscreteValueRule(const YAML::Node& rule_node) {
  if (!HasBaseRuleProperties(rule_node) || !rule_node[DiscreteValueRuleConstants::kValues].IsDefined()) {
    return false;
  }
  return std::all_of(rule_node[DiscreteValueRuleConstants::kValues].begin(),
                     rule_node[DiscreteValueRuleConstants::kValues].end(),
                     [](const auto& discrete_value_node) { return IsDiscreteValue(discrete_value_node); });
}

// Determines whether the `rule_node` corresponds to a api::rules::RangeValueRule description.
bool IsRangeValueRule(const YAML::Node& rule_node) {
  if (!HasBaseRuleProperties(rule_node) || !rule_node[RangeValueRuleConstants::kRanges].IsDefined()) {
    return false;
  }
  return std::all_of(rule_node[RangeValueRuleConstants::kRanges].begin(),
                     rule_node[RangeValueRuleConstants::kRanges].end(),
                     [](const auto& range_value_node) { return IsRangeValue(range_value_node); });
}

// Returns whether the `rule_node` refers to a RuleType::kDiscreteValueRuleType or
// RuleType::kRangeValueRuleType. When the rule type is ill-defined, RuleType::KUnknownRuleType is returned.
// @throws maliput::common::assertion_error when `rule_node` is ill-defined or empty.
RuleType EvaluateRuleType(const YAML::Node& rule_node) {
  MALIPUT_THROW_UNLESS(rule_node.IsMap());
  MALIPUT_THROW_UNLESS(rule_node.size());
  RuleType rule_type = IsDiscreteValueRule(rule_node) ? RuleType::kDiscreteValueRuleType : RuleType::kUnknownRuleType;
  rule_type = IsRangeValueRule(rule_node) ? RuleType::kRangeValueRuleType : rule_type;
  return rule_type;
}

// Returns a Rule::Id contained in the ID from `rule_node`.
// @throws maliput::common::assertion_error when ID is undefined within `rule_node`.
Rule::Id GetRuleIdFromYamlNode(const YAML::Node& rule_node) {
  MALIPUT_THROW_UNLESS(rule_node[RuleConstants::kId].IsDefined());
  return Rule::Id{rule_node[RuleConstants::kId].as<std::string>()};
}

// Returns a Rule::TypeId contained in the type from the `rule_node`.
// @throws maliput::common::assertion_error when the type is undefined within `rule_node`.
Rule::TypeId GetRuleTypeIdFromYamlNode(const YAML::Node& rule_node) {
  MALIPUT_THROW_UNLESS(rule_node[RuleConstants::kType].IsDefined());
  return Rule::TypeId{rule_node[RuleConstants::kType].as<std::string>()};
}

// Returns a api::SRange obtained from the `lane_node`.
// When the range is not defined within `lane_node`, the whole lane will be returned as api::SRange.
// @throws maliput::common::assertion_error when 'lane' is nullptr.
// @throws maliput::common::assertion_error when the lane range is not valid.
SRange GetSRange(const Lane* lane, const YAML::Node& lane_node) {
  MALIPUT_THROW_UNLESS(lane != nullptr);
  if (lane_node[RuleConstants::kSRange]) {
    const SRange s_range = lane_node[RuleConstants::kSRange].as<SRange>();
    MALIPUT_THROW_UNLESS(s_range.s0() >= 0);
    MALIPUT_THROW_UNLESS(s_range.s1() <= lane->length());
    return s_range;
  } else {
    return SRange(0, lane->length());
  }
}

// Returns a api::LaneSRange obtained from the `lane_s_range_node`.
// @throws maliput::common::assertion_error when 'road_geometry' is nullptr.
// @throws maliput::common::assertion_error when `lane_s_range_node` is ill-defined.
// @throws maliput::common::assertion_error when lane id is undefined within `lane_s_range_node`.
LaneSRange GetLaneSRangeFromYamlNode(const YAML::Node& lane_s_range_node, const api::RoadGeometry* road_geometry) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  MALIPUT_THROW_UNLESS(lane_s_range_node.IsMap());
  MALIPUT_THROW_UNLESS(lane_s_range_node[RuleConstants::kLaneId].IsDefined());
  const LaneId lane_id{lane_s_range_node[RuleConstants::kLaneId].as<std::string>()};
  const Lane* lane = road_geometry->ById().GetLane(lane_id);
  MALIPUT_THROW_UNLESS(lane != nullptr);
  const SRange s_range = GetSRange(lane, lane_s_range_node);
  return LaneSRange(lane_id, s_range);
}

// Returns a api::LaneSRoute contained in the zone from the `node`.
// @throws maliput::common::assertion_error when 'road_geometry' is nullptr.
// @throws maliput::common::assertion_error when the zone is ill-defined within `rule_node`.
LaneSRoute GetZoneFromYamlNode(const YAML::Node& rule_node, const api::RoadGeometry* road_geometry) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  MALIPUT_THROW_UNLESS(rule_node[RuleConstants::kZone].IsDefined());
  MALIPUT_THROW_UNLESS(rule_node[RuleConstants::kZone].IsSequence());
  std::vector<LaneSRange> zone;
  for (const auto& lane_s_range_node : rule_node[RuleConstants::kZone]) {
    zone.push_back(GetLaneSRangeFromYamlNode(lane_s_range_node, road_geometry));
  }
  return LaneSRoute{zone};
}

// Returns a Rule::RelatedRules contained in the related_rules from the `node`.
// @throws maliput::common::assertion_error when the related rules are ill-defined.
Rule::RelatedRules GetRelatedRuleFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[RuleConstants::kRelatedRules].IsSequence());
  Rule::RelatedRules related_rules{};
  for (const auto& map_keys_values : node[RuleConstants::kRelatedRules]) {
    for (const auto& pair_key_value : map_keys_values) {
      for (const auto& value : pair_key_value.second) {
        related_rules[pair_key_value.first.as<std::string>()].push_back(Rule::Id{{value.as<std::string>()}});
      }
    }
  }
  return related_rules;
}

// Returns a Rule::RelatedUniqueIds contained in the related_unique_ids from the `node`.
// @throws maliput::common::assertion_error when the related unique ids are ill-defined.
Rule::RelatedUniqueIds GetRelatedUniqueIdsFromYamlNode(const YAML::Node& node) {
  MALIPUT_THROW_UNLESS(node[RuleConstants::kRelatedUniqueIds].IsSequence());
  Rule::RelatedUniqueIds related_unique_ids{};
  for (const auto& map_keys_values : node[RuleConstants::kRelatedUniqueIds]) {
    for (const auto& pair_key_values : map_keys_values) {
      for (const auto& value : pair_key_values.second) {
        related_unique_ids[pair_key_values.first.as<std::string>()].push_back(api::UniqueId{value.as<std::string>()});
      }
    }
  }
  return related_unique_ids;
}

// Add a DiscreteValueRule to the `rulebook`.
// @throws maliput::common::assertion_error when 'rulebook' is nullptr.
// @throws maliput::common::assertion_error when 'road_geometry' is nullptr.
// @throws maliput::common::assertion_error when the `rule_node` is ill-defined.
// @throws maliput::common::assertion_error when the discrete value rules are ill-defined.
void AddDiscreteValueRule(ManualRulebook* rulebook, const YAML::Node& rule_node, const api::RoadGeometry* road_geometry,
                          const RuleRegistry& rule_registry) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  MALIPUT_THROW_UNLESS(rulebook != nullptr);
  MALIPUT_THROW_UNLESS(rule_node.IsMap());
  MALIPUT_THROW_UNLESS(rule_node[DiscreteValueRuleConstants::kValues].IsDefined());
  std::vector<DiscreteValueRule::DiscreteValue> discrete_values;
  for (const auto& discrete_value_node : rule_node[DiscreteValueRuleConstants::kValues]) {
    MALIPUT_THROW_UNLESS(discrete_value_node.IsMap());
    discrete_values.push_back(
        {GetSeverityFromYamlNode(discrete_value_node), GetRelatedRuleFromYamlNode(discrete_value_node),
         GetRelatedUniqueIdsFromYamlNode(discrete_value_node), GetValueFromYamlNode(discrete_value_node)});
  }
  rulebook->AddRule(
      rule_registry.BuildDiscreteValueRule(GetRuleIdFromYamlNode(rule_node), GetRuleTypeIdFromYamlNode(rule_node),
                                           GetZoneFromYamlNode(rule_node, road_geometry), discrete_values));
}

// Add a RangeValueRule to the `rulebook`.
// @throws maliput::common::assertion_error when 'rulebook' is nullptr.
// @throws maliput::common::assertion_error when 'road_geometry' is nullptr.
// @throws maliput::common::assertion_error when the `rule_node` is ill-defined.
// @throws maliput::common::assertion_error when the range value rules are ill-defined.
void AddRangeValueRule(ManualRulebook* rulebook, const YAML::Node& rule_node, const api::RoadGeometry* road_geometry,
                       const RuleRegistry& rule_registry) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  MALIPUT_THROW_UNLESS(rulebook != nullptr);
  MALIPUT_THROW_UNLESS(rule_node.IsMap());
  MALIPUT_THROW_UNLESS(rule_node[RangeValueRuleConstants::kRanges].IsDefined());
  std::vector<RangeValueRule::Range> range_values;
  for (const auto& range_value_node : rule_node[RangeValueRuleConstants::kRanges]) {
    MALIPUT_THROW_UNLESS(range_value_node.IsMap());
    const auto min_max = GetRangeMinMaxValuesFromYamlNode(range_value_node);
    range_values.push_back({GetSeverityFromYamlNode(range_value_node), GetRelatedRuleFromYamlNode(range_value_node),
                            GetRelatedUniqueIdsFromYamlNode(range_value_node),
                            GetDescriptionFromYamlNode(range_value_node), min_max.first, min_max.second});
  }
  rulebook->AddRule(rule_registry.BuildRangeValueRule(GetRuleIdFromYamlNode(rule_node),
                                                      GetRuleTypeIdFromYamlNode(rule_node),
                                                      GetZoneFromYamlNode(rule_node, road_geometry), range_values));
}

// Returns a api::rules::RoadRulebook created from `root_node`.
// @throws maliput::common::assertion_error when 'road_geometry' is nullptr.
// @throws maliput::common::assertion_error when the `root_node` is ill-defined.
std::unique_ptr<api::rules::RoadRulebook> BuildFrom(const api::RoadGeometry* road_geometry, const YAML::Node& root_node,
                                                    const RuleRegistry& rule_registry) {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  MALIPUT_THROW_UNLESS(root_node.IsMap());
  MALIPUT_THROW_UNLESS(root_node["RoadRulebook"].IsDefined());
  const YAML::Node& rulebook_node = root_node["RoadRulebook"];
  MALIPUT_THROW_UNLESS(rulebook_node.IsSequence());
  std::unique_ptr<ManualRulebook> rulebook = std::make_unique<ManualRulebook>();
  for (const YAML::Node& rule_node : rulebook_node) {
    switch (EvaluateRuleType(rule_node)) {
      case RuleType::kDiscreteValueRuleType:
        AddDiscreteValueRule(rulebook.get(), rule_node, road_geometry, rule_registry);
        break;
      case RuleType::kRangeValueRuleType:
        AddRangeValueRule(rulebook.get(), rule_node, road_geometry, rule_registry);
        break;
      default:
        MALIPUT_THROW_MESSAGE("Unknown Rule Type.");
        break;
    }
  }
  return rulebook;
}

}  // namespace

std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebook(const api::RoadGeometry* road_geometry,
                                                           const std::string& input,
                                                           const RuleRegistry& rule_registry) {
  return BuildFrom(road_geometry, YAML::Load(input), rule_registry);
}

std::unique_ptr<api::rules::RoadRulebook> LoadRoadRulebookFromFile(const api::RoadGeometry* road_geometry,
                                                                   const std::string& filename,
                                                                   const RuleRegistry& rule_registry) {
  return BuildFrom(road_geometry, YAML::LoadFile(filename), rule_registry);
}

}  // namespace maliput
