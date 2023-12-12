// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
  const api::rules::Rule::RelatedUniqueIds empty_related_unique_ids;
  std::vector<api::rules::DiscreteValueRule::DiscreteValue> values;
  for (const int severity : severities) {
    for (const std::string& state_value : state_values) {
      values.push_back(api::rules::DiscreteValueRule::DiscreteValue{severity, empty_related_rules,
                                                                    empty_related_unique_ids, state_value});
    }
  }
  return values;
}

}  // namespace

api::rules::Rule::TypeId DirectionUsageRuleTypeId() { return api::rules::Rule::TypeId("Direction-Usage Rule Type"); }

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
  return api::rules::Rule::TypeId("Vehicle-Stop-In-Zone-Behavior Rule Type");
}

api::rules::DiscreteValueRuleTypeAndValues BuildVehicleStopInZoneBehaviorRuleType() {
  const std::vector<std::string> state_values{"DoNotStop",       "5MinuteParking",      "30MinuteParking",
                                              "45MinuteParking", "1HourParking",        "2HourParking",
                                              "4HourParking",    "UnconstrainedParking"};
  return api::rules::DiscreteValueRuleTypeAndValues(
      VehicleStopInZoneBehaviorRuleTypeId(),
      GenerateEveryCombination({api::rules::Rule::State::kStrict}, state_values));
}

api::rules::Rule::TypeId SpeedLimitRuleTypeId() { return api::rules::Rule::TypeId{"Speed-Limit Rule Type"}; }

const char* RelatedRulesKeys::kYieldGroup{"Yield Group"};

const char* RelatedUniqueIdsKeys::kBulbGroup{"Bulb Group"};

}  // namespace maliput
