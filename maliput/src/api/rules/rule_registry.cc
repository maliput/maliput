#include "maliput/api/rules/rule_registry.h"

#include <utility>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

void RuleRegistry::RegisterRangeValueRule(const Rule::TypeId& type_id) {
  MALIPUT_THROW_UNLESS(
      range_rule_types_.find(type_id) == range_rule_types_.end());
  MALIPUT_THROW_UNLESS(
      discrete_rule_types_.find(type_id) == discrete_rule_types_.end());

  MALIPUT_THROW_UNLESS(range_rule_types_.emplace(type_id).second);
}

void RuleRegistry::RegisterDiscreteValueRule(
    const Rule::TypeId& type_id,
    const std::set<std::string>& all_possible_value_states) {
  MALIPUT_THROW_UNLESS(
      range_rule_types_.find(type_id) == range_rule_types_.end());
  MALIPUT_THROW_UNLESS(
      discrete_rule_types_.find(type_id) == discrete_rule_types_.end());
  MALIPUT_THROW_UNLESS(!all_possible_value_states.empty());

  MALIPUT_THROW_UNLESS(
      discrete_rule_types_.emplace(type_id, all_possible_value_states).second);
}

const std::set<Rule::TypeId>& RuleRegistry::RangeValueRuleTypes() const {
  return range_rule_types_;
}

const std::map<Rule::TypeId, std::set<std::string>>&
    RuleRegistry::DiscreteValueRuleTypes() const {
  return discrete_rule_types_;
}

RuleRegistry::QueryResult RuleRegistry::FindRuleTypeBy(
    const Rule::TypeId& type_id) const {
  RuleRegistry::QueryResult result;

  const auto range_value_rule_type_it = range_rule_types_.find(type_id);
  if (range_value_rule_type_it != range_rule_types_.end() ) {
    result.range_value_rule_type = *range_value_rule_type_it;
  }

  const auto discrete_value_rule_type_it = discrete_rule_types_.find(type_id);
  if (discrete_value_rule_type_it != discrete_rule_types_.end()) {
    result.discrete_value_rule_type = std::make_pair(
        discrete_value_rule_type_it->first, discrete_value_rule_type_it->second);
  }

  return result;
}


RangeValueRule RuleRegistry::BuildRangeValueRule(
    const Rule::Id& id, const Rule::TypeId& type_id,
    const LaneSRoute& zone, const std::vector<Rule::Id>& related_rules,
    const std::set<RangeValueRule::Range>& ranges) const {
  MALIPUT_THROW_UNLESS(
      range_rule_types_.find(type_id) != range_rule_types_.end());
  return RangeValueRule(id, type_id, zone, related_rules, ranges);
}

DiscreteValueRule RuleRegistry::BuildDiscreteValueRule(
    const Rule::Id& id, const Rule::TypeId& type_id,
    const LaneSRoute& zone, const std::vector<Rule::Id>& related_rules,
    const std::set<std::string>& value_states) const {
  MALIPUT_THROW_UNLESS(
      discrete_rule_types_.find(type_id) != discrete_rule_types_.end());
  const auto discrete_rule_type = discrete_rule_types_.find(type_id);
  for (const std::string& value_state : value_states) {
    MALIPUT_THROW_UNLESS(
        discrete_rule_type->second.find(value_state) !=
        discrete_rule_type->second.end());
  }

  return DiscreteValueRule(id, type_id, zone, related_rules, value_states);
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
