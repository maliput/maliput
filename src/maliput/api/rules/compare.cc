// BSD 3-Clause License
//
// Copyright (c) 2023-2026, Woven by Toyota.
// All rights reserved.
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

#include "maliput/api/rules/compare.h"

#include <optional>
#include <unordered_map>

#include "maliput/api/compare.h"

namespace maliput {
namespace api {
namespace rules {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

common::ComparisonResult<RuleStates> IsEqual(const RuleStates& a, const RuleStates& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  return {c.result()};
}

#pragma GCC diagnostic pop

common::ComparisonResult<DiscreteValueRule::DiscreteValue> IsEqual(const DiscreteValueRule::DiscreteValue& a,
                                                                   const DiscreteValueRule::DiscreteValue& b) {
  if (a.value != b.value) {
    return {"DiscreteValues are different: " + a.value + " != " + b.value};
  }
  return {std::nullopt};
}

common::ComparisonResult<std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>> IsEqual(
    const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& a,
    const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  for (const auto& discrete_value_rule_state : a) {
    MALIPUT_ADD_RESULT(c, IsEqual(b.at(discrete_value_rule_state.first), discrete_value_rule_state.second));
  }
  return {c.result()};
}

common::ComparisonResult<Phase> IsEqual(const Phase& a, const Phase& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id()", "b.id()", a.id(), b.id()));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MALIPUT_ADD_RESULT(c, IsEqual(a.rule_states(), b.rule_states()));
#pragma GCC diagnostic pop
  MALIPUT_ADD_RESULT(c, IsEqual(a.discrete_value_rule_states(), b.discrete_value_rule_states()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.bulb_states(), b.bulb_states()));
  return {c.result()};
}

common::ComparisonResult<PhaseRing::NextPhase> IsEqual(const PhaseRing::NextPhase& a, const PhaseRing::NextPhase& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id", "b.id", a.id, b.id));
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.duration_until.has_value()", "b.duration_until.has_value()",
                                     a.duration_until.has_value(), b.duration_until.has_value()));
  MALIPUT_ADD_RESULT(c, api::rules::IsEqual(a.duration_until, b.duration_until));
  return {c.result()};
}

common::ComparisonResult<std::vector<PhaseRing::NextPhase>> IsEqual(const std::vector<PhaseRing::NextPhase>& a,
                                                                    const std::vector<PhaseRing::NextPhase>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  if (a.size() == b.size()) {
    for (size_t i = 0; i < a.size(); ++i) {
      MALIPUT_ADD_RESULT(c, IsEqual(a[i], b[i]));
    }
  }
  return {c.result()};
}

common::ComparisonResult<RangeValueRule::Range> IsEqual(const rules::RangeValueRule::Range& a,
                                                        const rules::RangeValueRule::Range& b) {
  return {a != b ? std::make_optional<std::string>(
                       "Range with min: " + std::to_string(a.min) + " , max: " + std::to_string(a.max) +
                       " and description: " + a.description +
                       " is different from Range with min: " + std::to_string(b.min) +
                       " , max: " + std::to_string(b.max) + " and description: " + b.description)
                 : std::nullopt};
}

common::ComparisonResult<std::vector<rules::RangeValueRule::Range>> IsEqual(
    const std::vector<rules::RangeValueRule::Range>& a, const std::vector<rules::RangeValueRule::Range>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, IsEqual(a.at(i), b.at(i)));
  }
  return {c.result()};
}

common::ComparisonResult<RangeValueRule> IsEqual(const rules::RangeValueRule& a, const rules::RangeValueRule& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id()", "b.id()", a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.type_id()", "b.type_id()", a.type_id(), b.type_id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.states(), b.states()));
  return {c.result()};
}

common::ComparisonResult<DiscreteValueRule> IsEqual(const DiscreteValueRule& a, const DiscreteValueRule& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id()", "b.id()", a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.type_id()", "b.type_id()", a.type_id(), b.type_id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.states(), b.states()));
  return {c.result()};
}

common::ComparisonResult<std::vector<DiscreteValueRule::DiscreteValue>> IsEqual(
    const std::vector<DiscreteValueRule::DiscreteValue>& a, const std::vector<DiscreteValueRule::DiscreteValue>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, api::IsEqual("std::find(b.begin(), b.end(), a.at(i)) != b.end()", "true",
                                       std::find(b.begin(), b.end(), a.at(i)) != b.end(), true));
  }
  return {c.result()};
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

common::ComparisonResult<DirectionUsageRule::State::Type> IsEqual(DirectionUsageRule::State::Type a,
                                                                  DirectionUsageRule::State::Type b) {
  if (a != b) {
    return {"DirectionUsageRule::State::Type are different"};
  }
  return {std::nullopt};
}

common::ComparisonResult<DirectionUsageRule::State::Severity> IsEqual(DirectionUsageRule::State::Severity a,
                                                                      DirectionUsageRule::State::Severity b) {
  if (a != b) {
    return {"DirectionUsageRule::State::Severity are different"};
  }
  return {std::nullopt};
}

common::ComparisonResult<DirectionUsageRule::State> IsEqual(const DirectionUsageRule::State& a,
                                                            const DirectionUsageRule::State& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id()", "b.id()", a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.severity(), b.severity()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.type(), b.type()));
  return {c.result()};
}

common::ComparisonResult<std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State>> IsEqual(
    const std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State>& a,
    const std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  const std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State>& largest =
      (a.size() < b.size()) ? b : a;
  for (const auto& pair : largest) {
    const DirectionUsageRule::State::Id& key = pair.first;
    auto a_it = a.find(key);
    auto b_it = b.find(key);
    MALIPUT_ADD_RESULT(c, api::IsEqual("(a_it != a.cend())", "true", (a_it != a.cend()), true));
    MALIPUT_ADD_RESULT(c, api::IsEqual("(b_it != b.cend())", "true", (b_it != b.cend()), true));
    if ((a_it != a.cend()) && (b_it != b.cend())) {
      MALIPUT_ADD_RESULT(c, IsEqual(a_it->second, b_it->second));
    }
  }
  return {c.result()};
}

common::ComparisonResult<DirectionUsageRule> IsEqual(const DirectionUsageRule& a, const DirectionUsageRule& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id()", "b.id()", a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.states(), b.states()));
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.is_static()", "b.is_static()", a.is_static(), b.is_static()));
  if (a.is_static() && b.is_static()) {
    MALIPUT_ADD_RESULT(c, IsEqual(a.static_state(), b.static_state()));
  }
  return {c.result()};
}

common::ComparisonResult<rules::RightOfWayRule::ZoneType> IsEqual(rules::RightOfWayRule::ZoneType a,
                                                                  rules::RightOfWayRule::ZoneType b) {
  if (a != b) {
    return {"RightOfWayRule::ZoneType are different"};
  }
  return {std::nullopt};
}

common::ComparisonResult<rules::RightOfWayRule::State::Type> IsEqual(rules::RightOfWayRule::State::Type a,
                                                                     rules::RightOfWayRule::State::Type b) {
  if (a != b) {
    return {"RightOfWayRule::State::Type are different"};
  }
  return {std::nullopt};
}

common::ComparisonResult<std::vector<rules::RightOfWayRule::Id>> IsEqual(
    const std::vector<rules::RightOfWayRule::Id>& a, const std::vector<rules::RightOfWayRule::Id>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, IsEqual("a.at(i)", "b.at(i)", a.at(i), b.at(i)));
  }
  return {c.result()};
}

common::ComparisonResult<rules::RightOfWayRule::State> IsEqual(const rules::RightOfWayRule::State& a,
                                                               const rules::RightOfWayRule::State& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id()", "b.id()", a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.type(), b.type()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.yield_to(), b.yield_to()));
  return {c.result()};
}

common::ComparisonResult<std::unordered_map<rules::RightOfWayRule::State::Id, rules::RightOfWayRule::State>> IsEqual(
    const std::unordered_map<rules::RightOfWayRule::State::Id, rules::RightOfWayRule::State>& a,
    const std::unordered_map<rules::RightOfWayRule::State::Id, rules::RightOfWayRule::State>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  const std::unordered_map<rules::RightOfWayRule::State::Id, rules::RightOfWayRule::State>& largest =
      (a.size() < b.size()) ? b : a;
  for (const auto& pair : largest) {
    const rules::RightOfWayRule::State::Id& key = pair.first;
    auto a_it = a.find(key);
    auto b_it = b.find(key);
    MALIPUT_ADD_RESULT(c, api::IsEqual("(a_it != a.cend())", "true", (a_it != a.cend()), true));
    MALIPUT_ADD_RESULT(c, api::IsEqual("(b_it != b.cend())", "true", (b_it != b.cend()), true));
    if ((a_it != a.cend()) && (b_it != b.cend())) {
      MALIPUT_ADD_RESULT(c, IsEqual(a_it->second, b_it->second));
    }
  }
  return {c.result()};
}

common::ComparisonResult<rules::RightOfWayRule> IsEqual(const rules::RightOfWayRule& a,
                                                        const rules::RightOfWayRule& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id()", "b.id()", a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.zone_type(), b.zone_type()));
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.is_static()", "b.is_static()", a.is_static(), b.is_static()));
  if (a.is_static() && b.is_static()) {
    MALIPUT_ADD_RESULT(c, IsEqual(a.static_state(), b.static_state()));
  } else {
    MALIPUT_ADD_RESULT(c, IsEqual(a.states(), b.states()));
  }
  MALIPUT_ADD_RESULT(c, IsEqual(a.related_bulb_groups(), b.related_bulb_groups()));
  return {c.result()};
}

common::ComparisonResult<rules::RightOfWayRuleStateProvider::RightOfWayResult> IsEqual(
    const rules::RightOfWayRuleStateProvider::RightOfWayResult& a,
    const rules::RightOfWayRuleStateProvider::RightOfWayResult& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.state", "b.state", a.state, b.state));
  MALIPUT_ADD_RESULT(c,
                     api::IsEqual("a.next.has_value()", "b.next.has_value()", a.next.has_value(), b.next.has_value()));
  if (a.next.has_value() && b.next.has_value()) {
    MALIPUT_ADD_RESULT(c, api::IsEqual("a.next->state", "b.next->state", a.next->state, b.next->state));
    MALIPUT_ADD_RESULT(c, api::IsEqual("a.next->duration_until.has_value()", "b.next->duration_until.has_value()",
                                       a.next->duration_until.has_value(), b.next->duration_until.has_value()));
    if (a.next->duration_until.has_value() && b.next->duration_until.has_value()) {
      MALIPUT_ADD_RESULT(c, api::IsEqual("a.next->duration_until.value()", "b.next->duration_until.value()",
                                         a.next->duration_until.value(), b.next->duration_until.value()));
    }
  }
  return {c.result()};
}

common::ComparisonResult<rules::SpeedLimitRule::Severity> IsEqual(rules::SpeedLimitRule::Severity a,
                                                                  rules::SpeedLimitRule::Severity b) {
  if (a != b) {
    return {"SpeedLimitRule::Severity are different"};
  }
  return {std::nullopt};
}

common::ComparisonResult<rules::SpeedLimitRule> IsEqual(const rules::SpeedLimitRule& a,
                                                        const rules::SpeedLimitRule& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.id()", "b.id()", a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, IsEqual(a.severity(), b.severity()));
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.max()", "b.max()", a.max(), b.max()));
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.min()", "b.min()", a.min(), b.min()));
  return {c.result()};
}

#pragma GCC diagnostic pop

common::ComparisonResult<std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>> IsEqual(
    const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& a,
    const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.size()", "b.size()", a.size(), b.size()));
  if (a.size() == b.size()) {
    for (const auto& traffic_light_bulb_group : a) {
      const auto& b_it = b.find(traffic_light_bulb_group.first);
      MALIPUT_ADD_RESULT(c, api::IsEqual("(b_it != b.cend())", "true", (b_it != b.cend()), true));
      if (b_it == b.cend()) {
        break;
      }
      for (const BulbGroup::Id& bulb_group_id : traffic_light_bulb_group.second) {
        const auto& b_bulb_group_id_it = std::find(b_it->second.cbegin(), b_it->second.cend(), bulb_group_id);
        MALIPUT_ADD_RESULT(c, api::IsEqual("(b_bulb_group_id_it != b_it->second.cend())", "true",
                                           (b_bulb_group_id_it != b_it->second.cend()), true));
        if (b_bulb_group_id_it == b_it->second.cend()) {
          break;
        }
      }
    }
  }
  return {c.result()};
}

common::ComparisonResult<BulbColor> IsEqual(const BulbColor& a, const BulbColor& b) {
  if (a != b) {
    return {"BulbColors are different: " + std::string(BulbColorMapper().at(a)) +
            " != " + std::string(BulbColorMapper().at(b))};
  }
  return {std::nullopt};
}

common::ComparisonResult<BulbType> IsEqual(const BulbType& a, const BulbType& b) {
  if (a != b) {
    return {"BulbTypes are different: " + std::string(BulbTypeMapper().at(a)) +
            " != " + std::string(BulbTypeMapper().at(b))};
  }
  return {std::nullopt};
}

common::ComparisonResult<BulbState> IsEqual(const BulbState& a, const BulbState& b) {
  if (a != b) {
    return {"BulbStates are different: " + std::string(BulbStateMapper().at(a)) +
            " != " + std::string(BulbStateMapper().at(b))};
  }
  return {std::nullopt};
}

common::ComparisonResult<std::optional<BulbStates>> IsEqual(const std::optional<BulbStates>& a,
                                                            const std::optional<BulbStates>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.has_value()", "b.has_value()", a.has_value(), b.has_value()));
  if (a.has_value()) {
    MALIPUT_ADD_RESULT(c, api::IsEqual("a->size()", "b->size()", a->size(), b->size()));
    for (const auto& bulb_state : *a) {
      MALIPUT_ADD_RESULT(c, IsEqual(b->at(bulb_state.first), bulb_state.second));
    }
  }
  return {c.result()};
}

common::ComparisonResult<std::optional<double>> IsEqual(const std::optional<double>& a,
                                                        const std::optional<double>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.has_value()", "b.has_value()", a.has_value(), b.has_value()));
  if (a.has_value()) {
    MALIPUT_ADD_RESULT(c, api::IsEqual("a.value()", "b.value()", a.value(), b.value()));
  }
  return {c.result()};
}

common::ComparisonResult<Bulb::BoundingBox> IsEqual(const Bulb::BoundingBox& a, const Bulb::BoundingBox& b) {
  common::ComparisonResultCollector c;
  for (int i = 0; i < 3; ++i) {
    MALIPUT_ADD_RESULT(c, api::IsEqual("a.p_BMin[i]", "b.p_BMin[i]", a.p_BMin[i], b.p_BMin[i]));
    MALIPUT_ADD_RESULT(c, api::IsEqual("a.p_BMax[i]", "b.p_BMax[i]", a.p_BMax[i], b.p_BMax[i]));
  }
  return {c.result()};
}

common::ComparisonResult<Bulb> IsEqual(const Bulb* a, const Bulb* b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a->id()", "b->id()", a->id(), b->id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a->position_bulb_group(), b->position_bulb_group()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a->orientation_bulb_group(), b->orientation_bulb_group()));
  MALIPUT_ADD_RESULT(c, IsEqual(a->color(), b->color()));
  MALIPUT_ADD_RESULT(c, IsEqual(a->type(), b->type()));
  MALIPUT_ADD_RESULT(c, IsEqual(a->arrow_orientation_rad(), b->arrow_orientation_rad()));
  const std::vector<BulbState>& a_states = a->states();
  const std::vector<BulbState>& b_states = b->states();
  MALIPUT_ADD_RESULT(c, api::IsEqual("a->states().size()", "b->states().size()", a_states.size(), b_states.size()));
  const int smallest = std::min(a_states.size(), b_states.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, IsEqual(a_states.at(i), b_states.at(i)));
  }
  MALIPUT_ADD_RESULT(c, IsEqual(a->bounding_box(), b->bounding_box()));
  return {c.result()};
}

common::ComparisonResult<std::vector<const Bulb*>> IsEqual(const char* a_expression, const char* b_expression,
                                                           const std::vector<const Bulb*>& a,
                                                           const std::vector<const Bulb*>& b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual(a_expression, b_expression, a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, IsEqual(a.at(i), b.at(i)));
  }
  return {c.result()};
}

common::ComparisonResult<BulbGroup> IsEqual(const BulbGroup* a, const BulbGroup* b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a->id()", "b->id()", a->id(), b->id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a->position_traffic_light(), b->position_traffic_light()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a->orientation_traffic_light(), b->orientation_traffic_light()));
  MALIPUT_ADD_RESULT(c, IsEqual("a->bulbs()", "b->bulbs()", a->bulbs(), b->bulbs()));
  return {c.result()};
}

common::ComparisonResult<TrafficLight> IsEqual(const TrafficLight* a, const TrafficLight* b) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, api::IsEqual("a->id()", "b->id()", a->id(), b->id()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a->position_road_network(), b->position_road_network()));
  MALIPUT_ADD_RESULT(c, api::IsEqual(a->orientation_road_network(), b->orientation_road_network()));
  const std::vector<const BulbGroup*> bulb_groups_a = a->bulb_groups();
  const std::vector<const BulbGroup*> bulb_groups_b = b->bulb_groups();
  MALIPUT_ADD_RESULT(c, api::IsEqual("a->bulb_groups().size()", "b->bulb_groups().size()", bulb_groups_a.size(),
                                     bulb_groups_b.size()));
  const int smallest = std::min(bulb_groups_a.size(), bulb_groups_b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, IsEqual(bulb_groups_a.at(i), bulb_groups_b.at(i)));
  }
  return {c.result()};
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
