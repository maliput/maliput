// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota.
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
  MALIPUT_ADD_RESULT(c, api::IsEqual("a.duration_until.value()", "b.duration_until.value()", a.duration_until.value(),
                                     b.duration_until.value()));
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
#pragma GCC diagnostic pop

}  // namespace rules
}  // namespace api
}  // namespace maliput
