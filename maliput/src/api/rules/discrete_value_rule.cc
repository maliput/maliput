#include "maliput/api/rules/discrete_value_rule.h"

#include <algorithm>

namespace maliput {
namespace api {
namespace rules {

DiscreteValueRule::DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                                     const std::vector<Id> related_rules, const std::vector<DiscreteValue>& values)
    : Rule(id, type_id, zone, related_rules), values_(values) {
  MALIPUT_THROW_UNLESS(!values_.empty());
  for (const DiscreteValue& value : values_) {
    ValidateSeverity(value.severity);
    MALIPUT_THROW_UNLESS(std::count(values_.begin(), values_.end(), value) == 1);
  }
}

DiscreteValueRule::DiscreteValue MakeDiscreteValue(int severity, const std::string& value) {
  DiscreteValueRule::DiscreteValue discrete_value;
  discrete_value.severity = severity;
  discrete_value.value = value;
  return discrete_value;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
