#pragma once

#include <utility>
#include <vector>

#include "maliput/api/rules/discrete_value_rule.h"

namespace maliput {

/// Returns DirectionUsage rule type.
/// Rule::TypeId is equal to "DirectionUsageRuleType".
/// Values follow:
///
/// - WithS: Travel should proceed in the direction of the +S axis.
/// - AgainstS: Travel should proceed opposite the +S axis direction.
/// - Bidirectional: Travel is allowed both with the lane direction(+S) or against it.
/// - BidirectionalTurnOnly: Travel is allowed both with the lane direction(+S) or against it but should be limited in
///                          duration, e.g. when approaching turns.
/// - NoUse: Travel on this lane is prohibited.
/// - Parking: This lane is used to define a parking area.
/// - Undefined: There is no defined direction of travel on this lane.
///
/// With api::rules::Rule::State::kStrict strictness level.
std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildDirectionUsageRuleType();

/// Returns RightOfWay rule type.
/// Rule::TypeId is equal to "RighOfWayRuleType".
/// Discrete values are the combination of values and two strictness levels.
/// Values follow:
///
/// - kGo: The vehicle has right-of-way and may proceed if safe to do so.
/// - kStop: The vehicle does not have right-of-way and must stop.
/// - kStopThenGo: The vehicle must come to complete stop before entering
///   controlled zone, but may then proceed if safe.
///
/// Strictness levels are:
///
/// - api::rules::Rule::State::kStrict
/// - api::rules::Rule::State::kBestEffort
std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildRightOfWayRuleType();

/// Returns VehicleStopInZoneBehavior rule type.
/// Rule::TypeId is equal to "VehicleStopInZoneBehaviorRuleType".
/// Values follow:
///
/// - DoNotStop: The vehicle is not allowed to stop.
/// - 5MinuteParking: The vehicle is not allowed to park for five minutes.
/// - 30MinuteParking: The vehicle is not allowed to park for thirty minutes.
/// - 45MinuteParking: The vehicle is not allowed to park for forty-five minutes.
/// - 1HourParking: The vehicle is not allowed to park for one hour.
/// - 2HourParking: The vehicle is not allowed to park for two hours.
/// - 4HourParking: The vehicles is not allowed to park for four hours.
/// - UnrestrictedParking: vehicles are allowed to park without any restriction.
///
/// With api::rules::Rule::State::kStrict strictness level.
std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildVehicleStopInZoneBehaviorRuleType();

}  // namespace maliput
