#include "maliput/base/rule_registry.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule.h"

namespace maliput {
namespace {

// Returns a vector of api::rules::DiscreteValueRule::DiscreteValue whose
// values and severities are the combination of every item in `severities`
// and `state_values`.
std::vector<api::rules::DiscreteValueRule::DiscreteValue> CombineSeverityAndValues(
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

std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildDirectionUsageRuleType() {
  const api::rules::Rule::RelatedRules empty_related_rules;
  const std::vector<std::string> state_values{"WithS", "AgainstS", "Bidirectional", "BidirectionalTurnOnly",
                                              "NoUse", "Parking",  "Undefined"};
  return {api::rules::Rule::TypeId("DirectionUsageRuleType"),
          CombineSeverityAndValues({api::rules::Rule::State::kStrict}, state_values)};
}

std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildRightOfWayRuleType() {
  const std::vector<std::string> state_values{"Go", "Stop", "StopAndGo"};
  const std::vector<int> severities{api::rules::Rule::State::kStrict, api::rules::Rule::State::kBestEffort};
  return {api::rules::Rule::TypeId("RightOfWayRuleType"), CombineSeverityAndValues(severities, state_values)};
}

std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildVehicleStopInZoneBehaviorRuleType() {
  const std::vector<std::string> state_values{"DoNotStop",       "5MinuteParking",      "30MinuteParking",
                                              "45MinuteParking", "1HourParking",        "2HourParking",
                                              "4HourParking",    "UnconstrainedParking"};
  return {api::rules::Rule::TypeId("VehicleStopInZoneBehaviorRuleType"),
          CombineSeverityAndValues({api::rules::Rule::State::kStrict}, state_values)};
}

}  // namespace maliput
