#pragma once

#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/discrete_value_rule_state_provider.h"

namespace maliput {

/// A trivial implementation of api::rules::DiscreteValueRuleStateProvider.
class ManualDiscreteValueRuleStateProvider : public api::rules::DiscreteValueRuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManualDiscreteValueRuleStateProvider)

  ManualDiscreteValueRuleStateProvider() = default;
  ~ManualDiscreteValueRuleStateProvider() override = default;

  /// Adds a dynamic state to this provider.
  ///
  /// @throws std::logic_error When a Rule::Id with an ID of @p id already
  ///         exists in this provider.
  void AddState(const api::rules::Rule::Id& id, const api::rules::DiscreteValueRule::DiscreteValue& initial_state);

  /// Sets the dynamic state of a DiscreteValueRule within this provider.
  ///
  /// @throws std::out_of_range if no dynamic state with @p id exists in this
  ///         provider.
  void SetState(const api::rules::Rule::Id& id, const api::rules::DiscreteValueRule::DiscreteValue& state);

 private:
  drake::optional<api::rules::DiscreteValueRuleStateProvider::StateResult> DoGetState(
      const api::rules::Rule::Id& id) const final;

  std::unordered_map<api::rules::Rule::Id, api::rules::DiscreteValueRule::DiscreteValue> states_;
};

}  // namespace maliput
