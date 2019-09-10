#pragma once

#include <unordered_map>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {

/// An implementation of a manual api::rules::DiscreteValueRuleStateProvider.
/// This enables clients to directly add and set the states of
/// api::rules::DiscreteValueRule instances.
class ManualDiscreteValueRuleStateProvider : public api::rules::DiscreteValueRuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ManualDiscreteValueRuleStateProvider)

  /// Constructs a ManualDiscreteValueRuleStateProvider.
  ///
  /// @param rulebook A rulebook pointer to validate Rule::Id and their states.
  ///        It must not be nullptr.
  ///
  /// @throws maliput::common::assertion_error When `rulebook` is nullptr.
  explicit ManualDiscreteValueRuleStateProvider(const api::rules::RoadRulebook* rulebook)
      : api::rules::DiscreteValueRuleStateProvider(), rulebook_(rulebook) {
    MALIPUT_THROW_UNLESS(rulebook_ != nullptr);
  }

  ~ManualDiscreteValueRuleStateProvider() override = default;

  /// Registers a DiscreteValueRule with this provider.
  ///
  /// @p next_state and @p duration_until are not managed by this
  /// provider, i.e. it is client's responsibility to govern transitions from
  /// @p initial_state to @p next_state after @p duration_until (if provided)
  /// time.
  ///
  /// @param id DiscreteValueRule's ID. It must be recognized by the `rulebook`
  ///        specified at time of construction.
  /// @param initial_state The initial state of the rule. It must be one in
  ///        DiscreteValueRule::values().
  /// @param next_state The optional next state of the rule. It must be one in
  ///        DiscreteValueRule::values().
  /// @param duration_until If known, the estimated time until the transition to
  ///        the next state, relative to when this method is called. When
  ///        provided, it must be positive and @p next_state must not be nullopt.
  ///
  /// @throws std::logic_error When @p id already registered.
  /// @throws std::out_of_range When @p id is unrecognized by the `rulebook`
  ///         specified at time of construction.
  /// @throws maliput::common::assertion_error When @p initial_state does not
  ///         match any state in DiscreteValueRule::values().
  /// @throws maliput::common::assertion_error When @p next_state does not
  ///         match any state in DiscreteValueRule::values().
  /// @throws maliput::common::assertion_error When @p duration_until is not
  ///         positive or it is provided when @p next_state is nullopt.
  void Register(const api::rules::Rule::Id& id, const api::rules::DiscreteValueRule::DiscreteValue& initial_state,
                const drake::optional<api::rules::DiscreteValueRule::DiscreteValue>& next_state,
                const drake::optional<double>& duration_until);

  /// Sets the state, and optionally the next state, of a DiscreteValueRule
  /// within this provider.
  ///
  /// @p next_state and @p duration_until are not managed by this
  /// provider, i.e. it is client's responsibility to govern transitions from
  /// @p state to @p next_state after @p duration_until (if provided) time.
  ///
  /// @param id DiscreteValueRule's ID. It must be recognized by the `rulebook`
  ///        specified at time of construction.
  /// @param state The state of the rule. It must be one in
  ///        DiscreteValueRule::values().
  /// @param next_state The optional next state of the rule. It must be one in
  ///        DiscreteValueRule::values().
  /// @param duration_until If known, the estimated time until the transition to
  ///        the next state, relative to when this method is called. When
  ///        provided, it must be positive and @p next_state must not be nullopt.
  ///
  /// @throws std::logic_error When @p id already registered.
  /// @throws std::out_of_range When @p id is unrecognized by the `rulebook`
  ///         specified at time of construction.
  /// @throws maliput::common::assertion_error When @p state does not
  ///         match any state in DiscreteValueRule::values().
  /// @throws maliput::common::assertion_error When @p next_state does not
  ///         match any state in DiscreteValueRule::values().
  /// @throws maliput::common::assertion_error When @p duration_until is not
  ///         positive or it is provided when @p next_state is nullopt.
  void SetState(const api::rules::Rule::Id& id, const api::rules::DiscreteValueRule::DiscreteValue& state,
                const drake::optional<api::rules::DiscreteValueRule::DiscreteValue>& next_state,
                const drake::optional<double>& duration_until);

 private:
  drake::optional<api::rules::DiscreteValueRuleStateProvider::StateResult> DoGetState(
      const api::rules::Rule::Id& id) const final;

  // @throws maliput::common::assertion_error When @p state is unrecognized in
  //         @p discrete_value_rule's values.
  void ValidateRuleState(const api::rules::DiscreteValueRule& discrete_value_rule,
                         const api::rules::DiscreteValueRule::DiscreteValue& state) const;

  std::unordered_map<api::rules::Rule::Id, api::rules::DiscreteValueRuleStateProvider::StateResult> states_;
  const api::rules::RoadRulebook* rulebook_{nullptr};
};

}  // namespace maliput
