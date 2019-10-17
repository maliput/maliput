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
///  - ...
///
/// With api::rules::Rule::State::kStrict strictness level.
std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildDirectionUsageRuleType();

}  // namespace maliput
