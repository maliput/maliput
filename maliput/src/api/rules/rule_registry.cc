#include "maliput/api/rules/rule_registry.h"

#include <algorithm>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Convenient function to validate that `item` exists in `v`.
// @return true When `item` is in `v`.
template<typename T>
bool HasValue(const std::vector<T>& v, const T& item) {
  return std::find(v.begin(), v.end(), item) != v.end();
}

}  // namespace


void RuleRegistry::RegisterRangeValueRule(const Rule::TypeId& type_id,
                                          const std::vector<RangeValueRule::Range>& all_possible_ranges) {
  MALIPUT_THROW_UNLESS(FindRuleTypeBy(type_id) == drake::nullopt);
  MALIPUT_THROW_UNLESS(!all_possible_ranges.empty());
  for (const RangeValueRule::Range& range : all_possible_ranges) {
    MALIPUT_THROW_UNLESS(std::count(all_possible_ranges.begin(), all_possible_ranges.end(), range) == 1);
  }

  MALIPUT_THROW_UNLESS(range_rule_types_.emplace(type_id, all_possible_ranges).second);
}

void RuleRegistry::RegisterDiscreteValueRule(const Rule::TypeId& type_id,
                                             const std::vector<std::string>& all_possible_values) {
  MALIPUT_THROW_UNLESS(FindRuleTypeBy(type_id) == drake::nullopt);
  MALIPUT_THROW_UNLESS(!all_possible_values.empty());
  for (const std::string& value_state : all_possible_values) {
    MALIPUT_THROW_UNLESS(std::count(all_possible_values.begin(), all_possible_values.end(), value_state) == 1);
  }

  MALIPUT_THROW_UNLESS(discrete_rule_types_.emplace(type_id, all_possible_values).second);
}

const std::map<Rule::TypeId, std::vector<RangeValueRule::Range>>& RuleRegistry::RangeValueRuleTypes() const {
  return range_rule_types_;
}

const std::map<Rule::TypeId, std::vector<std::string>>& RuleRegistry::DiscreteValueRuleTypes() const {
  return discrete_rule_types_;
}

drake::optional<RuleRegistry::QueryResult> RuleRegistry::FindRuleTypeBy(const Rule::TypeId& type_id) const {
  const auto range_value_rule_type_it = range_rule_types_.find(type_id);
  if (range_value_rule_type_it != range_rule_types_.end()) {
    return {RuleRegistry::QueryResult{range_value_rule_type_it->first /* type_id */,
                                      range_value_rule_type_it->second /* range_values */,
                                      drake::nullopt /* discrete_values*/}};
  }

  const auto discrete_value_rule_type_it = discrete_rule_types_.find(type_id);
  if (discrete_value_rule_type_it != discrete_rule_types_.end()) {
    return {RuleRegistry::QueryResult{discrete_value_rule_type_it->first /* type_id */,
                                      drake::nullopt /* range_values */,
                                      discrete_value_rule_type_it->second /* discrete_values*/}};
  }

  return drake::nullopt;
}


RangeValueRule RuleRegistry::BuildRangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id,
                                                 const LaneSRoute& zone, const std::vector<Rule::Id>& related_rules,
                                                 const std::vector<RangeValueRule::Range>& ranges) const {
  const auto range_rule_type = range_rule_types_.find(type_id);
  MALIPUT_THROW_UNLESS(range_rule_type != range_rule_types_.end());
  for (const RangeValueRule::Range& range : ranges) {
    MALIPUT_THROW_UNLESS(HasValue(range_rule_type->second, range));
  }

  return RangeValueRule(id, type_id, zone, related_rules, ranges);
}

DiscreteValueRule RuleRegistry::BuildDiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id,
                                                       const LaneSRoute& zone,
                                                       const std::vector<Rule::Id>& related_rules,
                                                       const std::vector<std::string>& values) const {
  const auto discrete_rule_type = discrete_rule_types_.find(type_id);
  MALIPUT_THROW_UNLESS(discrete_rule_type != discrete_rule_types_.end());
  for (const std::string& value_state : values) {
    MALIPUT_THROW_UNLESS(HasValue(discrete_rule_type->second, value_state));
  }

  return DiscreteValueRule(id, type_id, zone, related_rules, values);
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
