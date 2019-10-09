#pragma once

#include <utility>
#include <vector>

#include "maliput/api/rules/discrete_value_rule.h"

namespace maliput {
/// Builds a DirectionUsageRule type.
/// Each DiscreteValue corresponds with one possible state of DirectionUsageRule:
/// kWithS, kAgainstS, kBidirectional, kBidirectionalTurnOnly, kNoUse, kParking.  .
/// This set api::rules::Rule::State:kStrict to all api::rules::DiscreteValueRule::DiscreteValue.
/// @return A std::pair conformed by a api::rules::Rule::TypeId with value DirectionUsageRuleType and a vector of
/// api::rules::DiscreteValueRule::DiscreteValue.
std::pair<api::rules::Rule::TypeId, std::vector<api::rules::DiscreteValueRule::DiscreteValue>>
BuildDirectionUsageRuleType();

}  // namespace maliput
