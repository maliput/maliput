#include "maliput/api/rules/discrete_value_rule.h"

#include <algorithm>

namespace maliput {
namespace api {
namespace rules {

DiscreteValueRule::DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                                     const std::vector<DiscreteValue>& values)
    : Rule(id, type_id, zone), values_(values) {
  MALIPUT_THROW_UNLESS(!values_.empty());
  for (const DiscreteValue& value : values_) {
    ValidateRelatedRules(value.related_rules);
    ValidateRelatedUniqueIds(value.related_unique_ids);
    ValidateSeverity(value.severity);
    MALIPUT_THROW_UNLESS(std::count(values_.begin(), values_.end(), value) == 1);
  }
}

DiscreteValueRule::DiscreteValue MakeDiscreteValue(int severity, const Rule::RelatedRules& related_rules,
                                                   const Rule::RelatedUniqueIds& related_unique_ids,
                                                   const std::string& value) {
  DiscreteValueRule::DiscreteValue discrete_value;
  discrete_value.severity = severity;
  discrete_value.related_rules = related_rules;
  discrete_value.related_unique_ids = related_unique_ids;
  discrete_value.value = value;
  return discrete_value;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
