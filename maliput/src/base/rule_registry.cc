#include "maliput/base/rule_registry.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule.h"

namespace maliput {

std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildDirectionUsageRuleType() {
  const api::rules::Rule::RelatedRules empty_related_rules;
  return {
      api::rules::Rule::TypeId("DirectionUsageRuleType"),
      {api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "WithS"),
       api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "AgainstS"),
       api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "Bidirectional"),
       api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "BidirectionalTurnOnly"),
       api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "NoUse"),
       api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "Parking"),
       api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, empty_related_rules, "Undefined")}};
}

maliput::api::rules::Rule::TypeId DirectionUsageRuleTypeId() {
  return maliput::api::rules::Rule::TypeId("DirectionUsageRuleType");
}

}  // namespace maliput
