#pragma once

#include <utility>
#include <vector>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule_registry.h"

namespace maliput {

/// Returns a Rule::TypeId whose string representation is
/// "Direction Usage Rule Type".
api::rules::Rule::TypeId DirectionUsageRuleTypeId();

/// Returns a direction usage rule type and its possible discrete values.
/// Rule::TypeId is initialized via DirectionUsageRuleTypeId().
/// Values:
///
/// - WithS: travel should proceed in the direction of the +S axis.
/// - AgainstS: travel should proceed opposite the +S axis direction.
/// - Bidirectional: travel is allowed both with the lane direction(+S) or
///   against it.
/// - BidirectionalTurnOnly: travel is allowed both with the lane direction(+S)
///   or against it but should be limited in duration, e.g. when approaching
///   turns.
/// - NoUse: Travel on this lane is prohibited.
/// - Parking: This lane is used to define a parking area.
/// - Undefined: There is no defined direction of travel on this lane.
///
/// With api::rules::Rule::State::kStrict strictness level. RelatedRules are
/// empty.
api::rules::DiscreteValueRuleTypeAndValues BuildDirectionUsageRuleType();

/// Returns a Rule::TypeId whose string representation is
/// "Right-Of-Way Rule Type".
api::rules::Rule::TypeId RightOfWayRuleTypeId();

/// Returns a right-of-way rule type and its possible discrete values.
/// Rule::TypeId is initialized via RightOfWayRuleTypeId().
/// Discrete values are the combination of values and two strictness levels.
/// Values:
///
/// - Go: the vehicle has right-of-way and may proceed across the rule's zone.
/// - Stop: the vehicle does not have right-of-way and must stop prior to
///   entering the rule's zone.
/// - StopThenGo: the vehicle must come to complete stop before entering
///   controlled zone, but may then proceed if safe.
///
/// Strictness levels are:
///
/// - api::rules::Rule::State::kStrict
/// - api::rules::Rule::State::kBestEffort
///
/// RelatedRules are empty.
api::rules::DiscreteValueRuleTypeAndValues BuildRightOfWayRuleType();

/// Returns a Rule::TypeId whose string representation is
/// "Vehicle Stop In Zone Behavior Rule Type".
api::rules::Rule::TypeId VehicleStopInZoneBehaviorRuleTypeId();

/// Returns a vehicle stop in zone behavior rule type and its possible discrete
/// values.
/// Rule::TypeId is initialized via VehicleStopInZoneBehaviorRuleTypeId().
/// Values:
///
/// - DoNotStop: the vehicle is not allowed to stop.
/// - 5MinuteStop: the vehicle is not allowed to park for more than five
///   minutes.
/// - 30MinuteStop: the vehicle is not allowed to park for more than five
///   thirty minutes.
/// - 45MinuteStop: the vehicle is not allowed to park for more than five
///   forty-five minutes.
/// - 1HourStop: the vehicle is not allowed to park for more than five
///   one hour.
/// - 2HourStop: the vehicle is not allowed to park for more than five
///   two hours.
/// - 4HourStop: the vehicles is not allowed to park for more than five
///   four hours.
/// - UnrestrictedParking: vehicles are allowed to park without any duration
///   restriction.
///
/// With api::rules::Rule::State::kStrict strictness level. RelatedRules are
/// empty.
api::rules::DiscreteValueRuleTypeAndValues BuildVehicleStopInZoneBehaviorRuleType();

/// Returns "YieldGroup", which is the key used in DiscreteValue::RelatedRules
/// by Right-Of-Way rules to yield to other rules.
std::string RightOfWayYieldGroup();

/// Returns "BulbGroupIds", which is the key to the `Rules::RelatedUniqueIds` of Right-Of-Way rules.
std::string RightOfWayBulbGroup();

}  // namespace maliput
