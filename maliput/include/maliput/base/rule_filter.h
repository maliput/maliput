#pragma once

#include <functional>
#include <vector>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule.h"

namespace maliput {

/// Convenient alias of a functor to filter api::rules::DiscreteValueRules based
/// on arbitrary criteria from a api::rules::RoadRulebook::QueryResult.
///
/// @see FilterRules() for further information.
using DiscreteValueRuleFilter = std::function<bool(const api::rules::DiscreteValueRule&)>;

/// Convenient alias of a functor to filter api::rules::RangeValueRule based on
/// arbitrary criteria from a api::rules::RoadRulebook::QueryResult.
///
/// @see FilterRules() for further information.
using RangeValueRuleFilter = std::function<bool(const api::rules::RangeValueRule&)>;

/// Returns `rules` after the application of `discrete_value_rules_filters` and
/// `range_value_rules_filters`.
///
/// Whether a rule is preserved or not is derived from applying each functor in
/// `discrete_value_rules_filters` and `range_value_rules_filters` to their
/// respective `rules` map (by type) as a chain of logic _ands_.
api::rules::RoadRulebook::QueryResults FilterRules(
    const api::rules::RoadRulebook::QueryResults& rules,
    const std::vector<DiscreteValueRuleFilter>& discrete_value_rules_filters,
    const std::vector<RangeValueRuleFilter>& range_value_rules_filters);

}  // namespace maliput
