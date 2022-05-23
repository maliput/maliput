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
#pragma once

#include <utility>
#include <vector>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule_registry.h"

namespace maliput {

/// Returns a Rule::TypeId whose string representation is
/// "Direction-Usage Rule Type".
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
/// "Vehicle-Stop-In-Zone-Behavior Rule Type".
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

/// Returns a maliput::api::rules::Rule::TypeId initialized with
/// "Speed-Limit Rule Type".
api::rules::Rule::TypeId SpeedLimitRuleTypeId();

/// Defines keys used in api::rules::Rule::RelatedRules.
struct RelatedRulesKeys {
  /// Key used by Right-Of-Way rules to yield to other rules.
  static const char* kYieldGroup;
};

/// Defines keys used in api::rules::Rule::RelatedUniqueIds.
struct RelatedUniqueIdsKeys {
  /// Key used by Right-Of-Way rules.
  static const char* kBulbGroup;
};

}  // namespace maliput
