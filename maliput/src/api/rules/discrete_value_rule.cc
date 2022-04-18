#include "maliput/api/rules/discrete_value_rule.h"

#include <algorithm>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

DiscreteValueRule::DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                                     const std::vector<DiscreteValue>& values)
    : Rule(id, type_id, zone), states_(values) {
  MALIPUT_VALIDATE(!states_.empty(),
                   "DiscreteValueRule(" + id.string() + ") has no DiscreteValueRule::DiscreteValues.");
  for (const DiscreteValue& value : states_) {
    ValidateRelatedRules(value.related_rules);
    ValidateRelatedUniqueIds(value.related_unique_ids);
    ValidateSeverity(value.severity);
    MALIPUT_VALIDATE(std::count(states_.begin(), states_.end(), value) == 1,
                     "DiscreteValueRule(" + id.string() + ") has duplicated DiscreteValueRule::DiscreteValues.");
  }
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
