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

}  // namespace maliput
