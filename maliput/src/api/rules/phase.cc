#include "maliput/api/rules/phase.h"

#include <utility>

namespace maliput {
namespace api {
namespace rules {

Phase::Phase(const Id& id, const RuleStates& rule_states, const DiscreteValueRuleStates& discrete_value_rule_states,
             drake::optional<BulbStates> bulb_states)
    : id_(id),
      rule_states_(std::move(rule_states)),
      discrete_value_rule_states_(std::move(discrete_value_rule_states)),
      bulb_states_(std::move(bulb_states)) {}

}  // namespace rules
}  // namespace api
}  // namespace maliput
