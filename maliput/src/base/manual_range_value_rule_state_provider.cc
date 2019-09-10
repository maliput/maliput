#include "maliput/base/manual_range_value_rule_state_provider.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace maliput {

void ManualRangeValueRuleStateProvider::ValidateRuleState(const api::rules::RangeValueRule& range_value_rule,
                                                          const api::rules::RangeValueRule::Range& state) const {
  if (std::find(range_value_rule.ranges().begin(), range_value_rule.ranges().end(), state) ==
      range_value_rule.ranges().end()) {
    MALIPUT_THROW_MESSAGE("Range is not in RangeValueRule " + range_value_rule.id().string() + "'s' ranges().");
  }
}

void ManualRangeValueRuleStateProvider::Register(const api::rules::Rule::Id& id,
                                                 const api::rules::RangeValueRule::Range& initial_state,
                                                 const drake::optional<api::rules::RangeValueRule::Range>& next_state,
                                                 const drake::optional<double>& duration_until) {
  const api::rules::RangeValueRule rule = rulebook_->GetRangeValueRule(id);
  ValidateRuleState(rule, initial_state);
  if (next_state.has_value()) {
    ValidateRuleState(rule, *next_state);
    if (duration_until.has_value()) {
      MALIPUT_THROW_UNLESS(*duration_until > 0.);
    }
  } else {
    MALIPUT_THROW_UNLESS(!duration_until.has_value());
  }

  api::rules::RangeValueRuleStateProvider::StateResult state_result;
  state_result.range_state = initial_state;
  if (next_state.has_value()) {
    state_result.next = {{*next_state, duration_until}};
  } else {
    state_result.next = drake::nullopt;
  }

  auto result = states_.emplace(id, state_result);
  if (!result.second) {
    throw std::logic_error("Attempted to add multiple rules with id " + id.string());
  }
}

void ManualRangeValueRuleStateProvider::SetState(const api::rules::Rule::Id& id,
                                                 const api::rules::RangeValueRule::Range& state,
                                                 const drake::optional<api::rules::RangeValueRule::Range>& next_state,
                                                 const drake::optional<double>& duration_until) {
  if (states_.find(id) == states_.end()) {
    throw std::out_of_range("Attempted to set state to an unregistered id " + id.string());
  }
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
  state_result.range_state = state;
  if (next_state.has_value()) {
    state_result.next = {{*next_state, duration_until}};
  }

  states_.at(id) = state_result;
}

drake::optional<api::rules::RangeValueRuleStateProvider::StateResult> ManualRangeValueRuleStateProvider::DoGetState(
    const api::rules::Rule::Id& id) const {
  const auto it = states_.find(id);
  if (it == states_.end()) {
    return drake::nullopt;
  }
  return it->second;
}

}  // namespace maliput
