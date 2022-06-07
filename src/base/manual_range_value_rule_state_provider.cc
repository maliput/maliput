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
#include "maliput/base/manual_range_value_rule_state_provider.h"

#include <algorithm>
#include <stdexcept>
#include <string>

#include "maliput/base/rule_filter.h"
#include "maliput/common/logger.h"

namespace maliput {

void ManualRangeValueRuleStateProvider::ValidateRuleState(const api::rules::RangeValueRule& range_value_rule,
                                                          const api::rules::RangeValueRule::Range& state) const {
  if (std::find(range_value_rule.states().begin(), range_value_rule.states().end(), state) ==
      range_value_rule.states().end()) {
    MALIPUT_THROW_MESSAGE("Range is not in RangeValueRule " + range_value_rule.id().string() + "'s' states().");
  }
}

void ManualRangeValueRuleStateProvider::SetState(const api::rules::Rule::Id& id,
                                                 const api::rules::RangeValueRule::Range& state,
                                                 const std::optional<api::rules::RangeValueRule::Range>& next_state,
                                                 const std::optional<double>& duration_until) {
  const api::rules::RangeValueRule rule = rulebook_->GetRangeValueRule(id);
  ValidateRuleState(rule, state);
  if (next_state.has_value()) {
    ValidateRuleState(rule, *next_state);
    if (duration_until.has_value()) {
      MALIPUT_THROW_UNLESS(*duration_until > 0.);
    }
  } else {
    MALIPUT_THROW_UNLESS(!duration_until.has_value());
  }

  api::rules::RangeValueRuleStateProvider::StateResult state_result;
  state_result.state = state;
  if (next_state.has_value()) {
    state_result.next = {{*next_state, duration_until}};
  }

  states_[id] = state_result;
}

std::optional<api::rules::RangeValueRuleStateProvider::StateResult> ManualRangeValueRuleStateProvider::DoGetState(
    const api::rules::Rule::Id& id) const {
  const auto it = states_.find(id);
  if (it == states_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<api::rules::RangeValueRuleStateProvider::StateResult> ManualRangeValueRuleStateProvider::DoGetState(
    const api::RoadPosition& road_position, const api::rules::Rule::TypeId& rule_type, double tolerance) const {
  MALIPUT_THROW_UNLESS(tolerance >= 0.);
  const auto query_result_rules = rulebook_->Rules();
  const RangeValueRuleFilter rule_type_filter = [&rule_type](const api::rules::RangeValueRule& rule) {
    return rule.type_id() == rule_type;
  };
  const RangeValueRuleFilter zone_filter = [&road_position, tolerance](const api::rules::RangeValueRule& rule) {
    const api::LaneSRange lane_s_range{road_position.lane->id(),
                                       api::SRange{road_position.pos.s(), road_position.pos.s()}};
    return rule.zone().Intersects(api::LaneSRoute({lane_s_range}), tolerance);
  };
  const auto filtered_rules = FilterRules(query_result_rules, {}, {rule_type_filter, zone_filter});
  if (filtered_rules.range_value_rules.size() > 1) {
    maliput::log()->warn(
        "For rule_type: {} and road_position: [LaneId: {}, LanePos: {}] there are more than one possible rules: ",
        rule_type.string(), road_position.lane->id(), road_position.pos.srh().to_str());
    for (const auto& rule : filtered_rules.range_value_rules) {
      maliput::log()->warn("\tRule id: {} matches with rule_type: {} and road_position: [LaneId: {}, LanePos: {}]",
                           rule.first.string(), rule_type.string(), road_position.lane->id(),
                           road_position.pos.srh().to_str());
    }
  }
  std::optional<api::rules::RangeValueRuleStateProvider::StateResult> current_state{std::nullopt};
  if (!filtered_rules.range_value_rules.empty()) {
    const auto state = states_.find(filtered_rules.range_value_rules.begin()->first);
    MALIPUT_THROW_UNLESS(state != states_.end());
    current_state = std::make_optional<>(state->second);
  }
  return current_state;
}

}  // namespace maliput
