#include "maliput/api/rules/rule_registry.h"

#include <algorithm>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Convenience function to validate that `discrete_value` exists in `discrete_values`.
// It does not check RelatedRules as it is customized by backends at build time.
// @return true When `discrete_value` is in `discrete_values`.
bool HasValue(const std::vector<DiscreteValueRule::DiscreteValue>& discrete_values,
              const DiscreteValueRule::DiscreteValue& discrete_value) {
  return std::find_if(discrete_values.begin(), discrete_values.end(), [discrete_value](const auto& dv) {
           return dv.severity == discrete_value.severity && dv.value == discrete_value.value;
         }) != discrete_values.end();
}

// Convenience function to validate that `range` exists in `ranges`.
// It does not check RelatedRules as it is customized by backends at build time.
// @return true When `range` is in `ranges`.
bool HasValue(const std::vector<RangeValueRule::Range>& ranges, const RangeValueRule::Range& range) {
  return std::find_if(ranges.begin(), ranges.end(), [range](const auto& r) {
           return r.severity == range.severity && r.min == range.min && r.max == range.max &&
                  r.description == range.description;
         }) != ranges.end();
}

}  // namespace

void RuleRegistry::RegisterRangeValueRule(const Rule::TypeId& type_id,
                                          const std::vector<RangeValueRule::Range>& all_possible_ranges) {
  MALIPUT_THROW_UNLESS(GetPossibleStatesOfRuleType(type_id) == drake::nullopt);
  MALIPUT_THROW_UNLESS(!all_possible_ranges.empty());
  for (const RangeValueRule::Range& range : all_possible_ranges) {
    MALIPUT_THROW_UNLESS(std::count(all_possible_ranges.begin(), all_possible_ranges.end(), range) == 1);
  }

  MALIPUT_THROW_UNLESS(range_rule_types_.emplace(type_id, all_possible_ranges).second);
}

void RuleRegistry::RegisterDiscreteValueRule(const Rule::TypeId& type_id,
                                             const std::vector<DiscreteValueRule::DiscreteValue>& all_possible_values) {
  MALIPUT_THROW_UNLESS(GetPossibleStatesOfRuleType(type_id) == drake::nullopt);
  MALIPUT_THROW_UNLESS(!all_possible_values.empty());
  for (const DiscreteValueRule::DiscreteValue& value_state : all_possible_values) {
    MALIPUT_THROW_UNLESS(std::count(all_possible_values.begin(), all_possible_values.end(), value_state) == 1);
  }

  MALIPUT_THROW_UNLESS(discrete_rule_types_.emplace(type_id, all_possible_values).second);
}

const std::map<Rule::TypeId, std::vector<RangeValueRule::Range>>& RuleRegistry::RangeValueRuleTypes() const {
  return range_rule_types_;
}

const std::map<Rule::TypeId, std::vector<DiscreteValueRule::DiscreteValue>>& RuleRegistry::DiscreteValueRuleTypes()
    const {
  return discrete_rule_types_;
}

drake::optional<RuleRegistry::QueryResult> RuleRegistry::GetPossibleStatesOfRuleType(
    const Rule::TypeId& type_id) const {
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
                                                 const LaneSRoute& zone,
                                                 const std::vector<RangeValueRule::Range>& ranges) const {
  const auto range_rule_type = range_rule_types_.find(type_id);
  MALIPUT_THROW_UNLESS(range_rule_type != range_rule_types_.end());
  for (const RangeValueRule::Range& range : ranges) {
    MALIPUT_THROW_UNLESS(HasValue(range_rule_type->second, range));
  }

  return RangeValueRule(id, type_id, zone, ranges);
}

DiscreteValueRule RuleRegistry::BuildDiscreteValueRule(
    const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
    const std::vector<DiscreteValueRule::DiscreteValue>& values) const {
  const auto discrete_rule_type = discrete_rule_types_.find(type_id);
  MALIPUT_THROW_UNLESS(discrete_rule_type != discrete_rule_types_.end());
  for (const DiscreteValueRule::DiscreteValue& value_state : values) {
    MALIPUT_THROW_UNLESS(HasValue(discrete_rule_type->second, value_state));
  }

  return DiscreteValueRule(id, type_id, zone, values);
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
