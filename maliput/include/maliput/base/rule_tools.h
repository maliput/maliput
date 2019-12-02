#pragma once

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/rule_registry.h"

namespace maliput {

/// Returns a vector of Rule::Ids that represents the yield group of a Right-Of-Way rule state.
/// @throws common::assertion_error When `discrete_value.related_rules` does not have `RightOfWayYieldGroup()` key.
std::vector<maliput::api::rules::Rule::Id> GetYieldGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value);

/// Returns the current yield group of `discrete_value_rule`.
/// @param discrete_value_rule A Right-Of-Way Rule Type rule.
/// @param state_provider A state provider to retrieve current `discrete_value_rule` state. It must not be nullptr.
/// @throws common::assertion_error When `discrete_value_rule.type_id()` is not RightOfWayRuleTypeId().
/// @throws common::assertion_error When `state_provider` is nullptr.
/// @throws common::assertion_error When `state_provider` does not hold any state for `discrete_value_rule`.
std::vector<maliput::api::rules::Rule::Id> GetCurrentYieldGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider);

/// Returns a vector of maliput::api::UniqueIds that represents the bulb group of a Right-Of-Way rule state.
/// @throws common::assertion_error When `discrete_value.related_unique_ids` does not have `RightOfWayBulbGroup()` key.
std::vector<maliput::api::UniqueId> GetBulbGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value);

/// Returns the current bulb group of `discrete_value_rule`.
/// @param discrete_value_rule A Right-Of-Way Rule Type rule.
/// @param state_provider A state provider to retrieve current `discrete_value_rule` state. It must not be nullptr.
/// @throws common::assertion_error When `discrete_value_rule.type_id()` is not RightOfWayRuleTypeId().
/// @throws common::assertion_error When `state_provider` is nullptr.
/// @throws common::assertion_error When `state_provider` does not hold any state for `discrete_value_rule`.
std::vector<maliput::api::UniqueId> GetCurrentBulbGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider);

/// Functor to filter by api::rules::Rule::TypeId.
struct RuleTypeFilter {
  RuleTypeFilter(const api::rules::Rule::TypeId& rule_type_in) : rule_type(rule_type_in) {}

  bool operator()(const api::rules::Rule& rule) { return rule.type_id() == rule_type; }

  api::rules::Rule::TypeId rule_type;
};

}  // namespace maliput
