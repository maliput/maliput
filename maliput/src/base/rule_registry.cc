#include "maliput/base/rule_registry.h"

#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule.h"

namespace maliput {
namespace {

// Returns a vector of api::rules::DiscreteValueRule::DiscreteValue whose
// values and severities are the combination of every item in `severities`
// and `state_values`.
std::vector<api::rules::DiscreteValueRule::DiscreteValue> GenerateEveryCombination(
    const std::vector<int>& severities, const std::vector<std::string>& state_values) {
  const api::rules::Rule::RelatedRules empty_related_rules;
  std::vector<api::rules::DiscreteValueRule::DiscreteValue> values;
  for (const int severity : severities) {
    for (const std::string& state_value : state_values) {
      values.push_back(api::rules::MakeDiscreteValue(severity, empty_related_rules, state_value));
    }
  }
  return values;
}

}  // namespace

api::rules::Rule::TypeId DirectionUsageRuleTypeId() { return api::rules::Rule::TypeId("Direction Usage Rule Type"); }

api::rules::DiscreteValueRuleTypeAndValues BuildDirectionUsageRuleType() {
  const std::vector<std::string> state_values{"WithS", "AgainstS", "Bidirectional", "BidirectionalTurnOnly",
                                              "NoUse", "Parking",  "Undefined"};
  return api::rules::DiscreteValueRuleTypeAndValues(
      DirectionUsageRuleTypeId(), GenerateEveryCombination({api::rules::Rule::State::kStrict}, state_values));
}

api::rules::Rule::TypeId RightOfWayRuleTypeId() { return api::rules::Rule::TypeId("Right-Of-Way Rule Type"); }

api::rules::DiscreteValueRuleTypeAndValues BuildRightOfWayRuleType() {
  const std::vector<std::string> state_values{"Go", "Stop", "StopAndGo"};
  const std::vector<int> severities{api::rules::Rule::State::kStrict, api::rules::Rule::State::kBestEffort};
  return api::rules::DiscreteValueRuleTypeAndValues(RightOfWayRuleTypeId(),
                                                    GenerateEveryCombination(severities, state_values));
}

api::rules::Rule::TypeId VehicleStopInZoneBehaviorRuleTypeId() {
  return api::rules::Rule::TypeId("Vehicle Stop In Zone Behavior Rule Type");
}

api::rules::DiscreteValueRuleTypeAndValues BuildVehicleStopInZoneBehaviorRuleType() {
  const std::vector<std::string> state_values{"DoNotStop",       "5MinuteParking",      "30MinuteParking",
                                              "45MinuteParking", "1HourParking",        "2HourParking",
                                              "4HourParking",    "UnconstrainedParking"};
  return api::rules::DiscreteValueRuleTypeAndValues(
      VehicleStopInZoneBehaviorRuleTypeId(),
      GenerateEveryCombination({api::rules::Rule::State::kStrict}, state_values));
}

std::string YieldGroup() { return "YieldGroup"; }

}  // namespace maliput
