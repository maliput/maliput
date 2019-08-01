#include "maliput/api/rules/rule_registry.h"

#include <algorithm>
#include <utility>

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


void RuleRegistry::RegisterRangeValueRule(const Rule::TypeId& type_id) {
  MALIPUT_THROW_UNLESS(!HasValue(range_rule_types_, type_id));
  MALIPUT_THROW_UNLESS(
      discrete_rule_types_.find(type_id) == discrete_rule_types_.end());

  range_rule_types_.push_back(type_id);
}

void RuleRegistry::RegisterDiscreteValueRule(
    const Rule::TypeId& type_id,
    const std::vector<std::string>& all_possible_value_states) {
  MALIPUT_THROW_UNLESS(!HasValue(range_rule_types_, type_id));
  MALIPUT_THROW_UNLESS(
      discrete_rule_types_.find(type_id) == discrete_rule_types_.end());
  MALIPUT_THROW_UNLESS(!all_possible_value_states.empty());
  for (const std::string& value_state : all_possible_value_states) {
    MALIPUT_THROW_UNLESS(
        std::count(all_possible_value_states.begin(),
                   all_possible_value_states.end(),
                   value_state) ==
        1);
  }

  MALIPUT_THROW_UNLESS(
      discrete_rule_types_.emplace(type_id, all_possible_value_states).second);
}

const std::vector<Rule::TypeId>& RuleRegistry::RangeValueRuleTypes() const {
  return range_rule_types_;
}

const std::map<Rule::TypeId, std::vector<std::string>>&
    RuleRegistry::DiscreteValueRuleTypes() const {
  return discrete_rule_types_;
}

RuleRegistry::QueryResult RuleRegistry::FindRuleTypeBy(
    const Rule::TypeId& type_id) const {
  RuleRegistry::QueryResult result;

  const auto range_value_rule_type_it =
      std::find(range_rule_types_.begin(), range_rule_types_.end(), type_id);
  if (range_value_rule_type_it != range_rule_types_.end()) {
    result.range_value_rule_type = *range_value_rule_type_it;
  }

  const auto discrete_value_rule_type_it = discrete_rule_types_.find(type_id);
  if (discrete_value_rule_type_it != discrete_rule_types_.end()) {
    result.discrete_value_rule_type = std::make_pair(
        discrete_value_rule_type_it->first,
        discrete_value_rule_type_it->second);
  }

  return result;
}


RangeValueRule RuleRegistry::BuildRangeValueRule(
    const Rule::Id& id, const Rule::TypeId& type_id,
    const LaneSRoute& zone, const std::vector<Rule::Id>& related_rules,
    const std::vector<RangeValueRule::Range>& ranges) const {
  MALIPUT_THROW_UNLESS(HasValue(range_rule_types_, type_id));

  return RangeValueRule(id, type_id, zone, related_rules, ranges);
}

DiscreteValueRule RuleRegistry::BuildDiscreteValueRule(
    const Rule::Id& id, const Rule::TypeId& type_id,
    const LaneSRoute& zone, const std::vector<Rule::Id>& related_rules,
    const std::vector<std::string>& value_states) const {
  MALIPUT_THROW_UNLESS(
      discrete_rule_types_.find(type_id) != discrete_rule_types_.end());
  const auto discrete_rule_type = discrete_rule_types_.find(type_id);
  for (const std::string& value_state : value_states) {
    MALIPUT_THROW_UNLESS(HasValue(discrete_rule_type->second, value_state));
  }

  return DiscreteValueRule(id, type_id, zone, related_rules, value_states);
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
