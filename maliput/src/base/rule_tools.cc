#include "maliput/base/rule_tools.h"

#include <optional>

#include "maliput/api/rules/state_provider_result.h"
#include "maliput/base/rule_registry.h"

namespace maliput {

namespace {

// Returns the current state for a given Right-Of-Way Rule.
maliput::api::rules::DiscreteValueRule::DiscreteValue GetCurrentStateValue(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  MALIPUT_THROW_UNLESS(state_provider != nullptr);
  MALIPUT_THROW_UNLESS(discrete_value_rule.type_id() == RightOfWayRuleTypeId());
  const std::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult> state_result{
      state_provider->GetState(discrete_value_rule.id())};
  MALIPUT_THROW_UNLESS(state_result != std::nullopt);
  return state_result->state;
}

}  // namespace

std::vector<maliput::api::rules::Rule::Id> GetYieldGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
  MALIPUT_THROW_UNLESS(discrete_value.related_rules.find(RelatedRulesKeys::kYieldGroup) !=
                       discrete_value.related_rules.end());
  return discrete_value.related_rules.at(RelatedRulesKeys::kYieldGroup);
}

std::vector<maliput::api::rules::Rule::Id> GetCurrentYieldGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  return GetYieldGroup(GetCurrentStateValue(discrete_value_rule, state_provider));
}

std::vector<maliput::api::UniqueId> GetBulbGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
  MALIPUT_THROW_UNLESS(discrete_value.related_unique_ids.find(RelatedUniqueIdsKeys::kBulbGroup) !=
                       discrete_value.related_unique_ids.end());
  return discrete_value.related_unique_ids.at(RelatedUniqueIdsKeys::kBulbGroup);
}

std::vector<maliput::api::UniqueId> GetCurrentBulbGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  return GetBulbGroup(GetCurrentStateValue(discrete_value_rule, state_provider));
}

}  // namespace maliput
