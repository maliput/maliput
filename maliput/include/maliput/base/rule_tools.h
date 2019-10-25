#pragma once

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/rule_registry.h"

using namespace maliput::api::rules;
namespace maliput {

/// TODO docstring
std::vector<Rule::Id> GetYieldGroup(const api::rules::DiscreteValueRule::DiscreteValue& discrete_value);

/// TODO docstring
std::vector<Rule::Id> GetCurrentYieldGroup(const api::rules::DiscreteValueRule& discrete_value_rule,
                                           const api::rules::DiscreteValueRuleStateProvider* state_provider);

}  // namespace maliput
