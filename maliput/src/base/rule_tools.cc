#include "maliput/base/rule_tools.h"
#include "maliput/api/rules/state_provider_result.h"
#include "maliput/base/rule_registry.h"

#include "drake/common/drake_optional.h"

namespace maliput {

std::vector<maliput::api::rules::Rule::Id> GetYieldGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
  MALIPUT_THROW_UNLESS(discrete_value.related_rules.find(RightOfWayYieldGroup()) != discrete_value.related_rules.end());
  return discrete_value.related_rules.at(RightOfWayYieldGroup());
}

std::vector<maliput::api::rules::Rule::Id> GetCurrentYieldGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  MALIPUT_THROW_UNLESS(state_provider != nullptr);
  drake::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult> state_result{
      state_provider->GetState(discrete_value_rule.id())};
  MALIPUT_THROW_UNLESS(state_result != drake::nullopt);
  return GetYieldGroup(state_result->state);
}

}  // namespace maliput
