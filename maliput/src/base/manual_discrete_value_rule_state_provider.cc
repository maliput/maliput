#include "maliput/base/manual_discrete_value_rule_state_provider.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace maliput {

void ManualDiscreteValueRuleStateProvider::ValidateRuleState(
    const api::rules::DiscreteValueRule& discrete_value_rule,
    const api::rules::DiscreteValueRule::DiscreteValue& state) const {
  if (std::find(discrete_value_rule.values().begin(), discrete_value_rule.values().end(), state) ==
      discrete_value_rule.values().end()) {
    MALIPUT_THROW_MESSAGE("DiscreteValue is not in DiscreteValueRule " + discrete_value_rule.id().string() +
                          "'s' values().");
  }
}

void ManualDiscreteValueRuleStateProvider::Register(
    const api::rules::Rule::Id& id, const api::rules::DiscreteValueRule::DiscreteValue& initial_state,
    const drake::optional<api::rules::DiscreteValueRule::DiscreteValue>& next_state,
    const drake::optional<double>& duration_until) {
  const api::rules::DiscreteValueRule rule = rulebook_->GetDiscreteValueRule(id);
  ValidateRuleState(rule, initial_state);
  if (next_state.has_value()) {
    ValidateRuleState(rule, *next_state);
    if (duration_until.has_value()) {
      MALIPUT_THROW_UNLESS(*duration_until > 0.);
    }
  } else {
    MALIPUT_THROW_UNLESS(!duration_until.has_value());
  }

  api::rules::DiscreteValueRuleStateProvider::StateResult state_result;
  state_result.value_state = initial_state;
  if (next_state.has_value()) {
    state_result.next = {{*next_state, duration_until}};
  }

  auto result = states_.emplace(id, state_result);
  if (!result.second) {
    throw std::logic_error("Attempted to add multiple rules with id " + id.string());
  }
}

void ManualDiscreteValueRuleStateProvider::SetState(
    const api::rules::Rule::Id& id, const api::rules::DiscreteValueRule::DiscreteValue& state,
    const drake::optional<api::rules::DiscreteValueRule::DiscreteValue>& next_state,
    const drake::optional<double>& duration_until) {
  if (states_.find(id) == states_.end()) {
    throw std::out_of_range("Attempted to set state to an unregistered id " + id.string());
  }
  const api::rules::DiscreteValueRule rule = rulebook_->GetDiscreteValueRule(id);
  ValidateRuleState(rule, state);
  if (next_state.has_value()) {
    ValidateRuleState(rule, *next_state);
    if (duration_until.has_value()) {
      MALIPUT_THROW_UNLESS(*duration_until > 0.);
    }
  } else {
    MALIPUT_THROW_UNLESS(!duration_until.has_value());
  }

  api::rules::DiscreteValueRuleStateProvider::StateResult state_result;
  state_result.value_state = state;
  if (next_state.has_value()) {
    state_result.next = {{*next_state, duration_until}};
  }

  states_.at(id) = state_result;
}

drake::optional<api::rules::DiscreteValueRuleStateProvider::StateResult>
ManualDiscreteValueRuleStateProvider::DoGetState(const api::rules::Rule::Id& id) const {
  const auto it = states_.find(id);
  if (it == states_.end()) {
    return drake::nullopt;
  }
  return it->second;
}

}  // namespace maliput
