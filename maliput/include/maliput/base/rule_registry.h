#pragma once

#include <utility>
#include <vector>

#include "maliput/api/rules/discrete_value_rule.h"

namespace maliput {
/// Returns DirectionUsage rule type.
/// Rule::TypeId is equal to "DirectionUsageRuleType".
/// Discrete values are listed below:
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

/// @returns a maliput::api::rules::Rule::TypeId initialized with
/// "DirectionUsageRuleType".
maliput::api::rules::Rule::TypeId DirectionUsageRuleTypeId();

}  // namespace maliput
