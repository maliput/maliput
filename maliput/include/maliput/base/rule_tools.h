#pragma once

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/rule_registry.h"

namespace maliput {

/// Returns a Rule::Id vector containing all the related rules of type YieldGroup from a given `discrete_value`.
std::vector<maliput::api::rules::Rule::Id> GetYieldGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value);

/// Returns a Rule::Id vector containing all the related rules of type YieldGroup from a
/// given `discrete_value_rule` and `state_provider`.
std::vector<maliput::api::rules::Rule::Id> GetCurrentYieldGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider);

}  // namespace maliput
