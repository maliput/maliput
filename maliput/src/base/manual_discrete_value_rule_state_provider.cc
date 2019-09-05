#include "maliput/base/manual_discrete_value_rule_state_provider.h"

#include <stdexcept>

namespace maliput {

void ManualDiscreteValueRuleStateProvider::AddState(const api::rules::Rule::Id& id,
                                                    const api::rules::DiscreteValueRule::DiscreteValue& initial_state) {
  auto result = states_.emplace(id, initial_state);
  if (!result.second) {
    throw std::logic_error("Attempted to add multiple rules with id " + id.string());
  }
}

void ManualDiscreteValueRuleStateProvider::SetState(const api::rules::Rule::Id& id,
                                                    const api::rules::DiscreteValueRule::DiscreteValue& state) {
  states_.at(id) = state;
}

drake::optional<api::rules::DiscreteValueRuleStateProvider::StateResult>
ManualDiscreteValueRuleStateProvider::DoGetState(const api::rules::Rule::Id& id) const {
  const auto it = states_.find(id);
  if (it == states_.end()) {
    return drake::nullopt;
  }
  return api::rules::DiscreteValueRuleStateProvider::StateResult{it->second /* state_range */,
                                                                 drake::nullopt /* next */};
}

}  // namespace maliput
