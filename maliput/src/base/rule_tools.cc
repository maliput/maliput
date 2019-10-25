#include "maliput/base/rule_tools.h"
#include "maliput/api/rules/state_provider_result.h"
#include "maliput/base/rule_registry.h"

#include "drake/common/drake_optional.h"  //--CHECK

namespace maliput {

std::vector<Rule::Id> GetYieldGroup(const api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
  MALIPUT_THROW_UNLESS(discrete_value.related_rules.find(RightOfWayYieldGroup()) != discrete_value.related_rules.end());
  return discrete_value.related_rules.at(RightOfWayYieldGroup());
}

std::vector<Rule::Id> GetCurrentYieldGroup(const api::rules::DiscreteValueRule& discrete_value_rule,
                                           const api::rules::DiscreteValueRuleStateProvider* state_provider) {
  MALIPUT_THROW_UNLESS(state_provider != nullptr);
  drake::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult> state_result{
      state_provider->GetState(discrete_value_rule.id())};
  MALIPUT_THROW_UNLESS(state_result != drake::nullopt);
  return GetYieldGroup(state_result->state);
}

}  // namespace maliput
