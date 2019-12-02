#include "maliput/base/rule_tools.h"
#include "maliput/api/rules/state_provider_result.h"
#include "maliput/base/rule_registry.h"

#include "drake/common/drake_optional.h"

namespace maliput {

namespace {

// Returns the current state for a given Right-Of-Way Rule.
maliput::api::rules::DiscreteValueRule::DiscreteValue GetCurrentStateValue(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  MALIPUT_THROW_UNLESS(state_provider != nullptr);
  MALIPUT_THROW_UNLESS(discrete_value_rule.type_id() == RightOfWayRuleTypeId());
  const drake::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult> state_result{
      state_provider->GetState(discrete_value_rule.id())};
  MALIPUT_THROW_UNLESS(state_result != drake::nullopt);
  return state_result->state;
}

}  // namespace

std::vector<maliput::api::rules::Rule::Id> GetYieldGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
  MALIPUT_THROW_UNLESS(discrete_value.related_rules.find(RightOfWayYieldGroup()) != discrete_value.related_rules.end());
  return discrete_value.related_rules.at(RightOfWayYieldGroup());
}

std::vector<maliput::api::rules::Rule::Id> GetCurrentYieldGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  return GetYieldGroup(GetCurrentStateValue(discrete_value_rule, state_provider));
}

std::vector<maliput::api::UniqueId> GetBulbGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
  MALIPUT_THROW_UNLESS(discrete_value.related_unique_ids.find(RightOfWayBulbGroup()) !=
                       discrete_value.related_unique_ids.end());
  return discrete_value.related_unique_ids.at(RightOfWayBulbGroup());
}

std::vector<maliput::api::UniqueId> GetCurrentBulbGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  return GetBulbGroup(GetCurrentStateValue(discrete_value_rule, state_provider));
}

}  // namespace maliput
