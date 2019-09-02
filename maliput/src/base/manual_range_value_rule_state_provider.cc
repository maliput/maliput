#include "maliput/base/manual_range_value_rule_state_provider.h"

#include <stdexcept>

namespace maliput {

void ManualRangeValueRuleStateProvider::AddState(const api::rules::Rule::Id& id,
                                                 const api::rules::RangeValueRule::Range& initial_state) {
  auto result = states_.emplace(id, initial_state);
  if (!result.second) {
    throw std::logic_error("Attempted to add multiple rules with id " + id.string());
  }
}

void ManualRangeValueRuleStateProvider::SetState(const api::rules::Rule::Id& id,
                                                 const api::rules::RangeValueRule::Range& state) {
  states_.at(id) = state;
}

drake::optional<api::rules::RangeValueRuleStateProvider::StateResult> ManualRangeValueRuleStateProvider::DoGetState(
    const api::rules::Rule::Id& id) const {
  auto it = states_.find(id);
  if (it == states_.end()) {
    return drake::nullopt;
  }
  return api::rules::RangeValueRuleStateProvider::StateResult{it->second /* state_range */, drake::nullopt /* next */};
}

}  // namespace maliput
