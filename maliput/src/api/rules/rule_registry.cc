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
#include "maliput/api/rules/rule_registry.h"

#include <algorithm>
#include <optional>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Convenience function to validate that `discrete_value` exists in `discrete_values`.
// It does not check RelatedRules as it is customized by backends at build time.
// @return true When `discrete_value` is in `discrete_values`.
bool HasValue(const RuleRegistry::QueryResult::DiscreteValues& discrete_values,
              const DiscreteValueRule::DiscreteValue& discrete_value) {
  return std::find_if(discrete_values.begin(), discrete_values.end(), [discrete_value](const auto& dv) {
           return dv.severity == discrete_value.severity && dv.value == discrete_value.value;
         }) != discrete_values.end();
}

// Convenience function to validate that `range` exists in `ranges`.
// It does not check RelatedRules as it is customized by backends at build time.
// @return true When `range` is in `ranges`.
bool HasValue(const RuleRegistry::QueryResult::Ranges& ranges, const RangeValueRule::Range& range) {
  return std::find_if(ranges.begin(), ranges.end(), [range](const auto& r) {
           return r.severity == range.severity && r.min == range.min && r.max == range.max &&
                  r.description == range.description;
         }) != ranges.end();
}

}  // namespace

void RuleRegistry::RegisterRangeValueRule(const Rule::TypeId& type_id,
                                          const RuleRegistry::QueryResult::Ranges& all_possible_ranges) {
  MALIPUT_THROW_UNLESS(GetPossibleStatesOfRuleType(type_id) == std::nullopt);
  MALIPUT_THROW_UNLESS(!all_possible_ranges.empty());
  for (const RangeValueRule::Range& range : all_possible_ranges) {
    MALIPUT_THROW_UNLESS(std::count(all_possible_ranges.begin(), all_possible_ranges.end(), range) == 1);
  }

  MALIPUT_THROW_UNLESS(range_rule_types_.emplace(type_id, all_possible_ranges).second);
}

void RuleRegistry::RegisterDiscreteValueRule(const Rule::TypeId& type_id,
                                             const RuleRegistry::QueryResult::DiscreteValues& all_possible_values) {
  MALIPUT_THROW_UNLESS(GetPossibleStatesOfRuleType(type_id) == std::nullopt);
  MALIPUT_THROW_UNLESS(!all_possible_values.empty());
  for (const DiscreteValueRule::DiscreteValue& value_state : all_possible_values) {
    MALIPUT_THROW_UNLESS(std::count(all_possible_values.begin(), all_possible_values.end(), value_state) == 1);
  }

  MALIPUT_THROW_UNLESS(discrete_rule_types_.emplace(type_id, all_possible_values).second);
}

const std::map<Rule::TypeId, RuleRegistry::QueryResult::Ranges>& RuleRegistry::RangeValueRuleTypes() const {
  return range_rule_types_;
}

const std::map<Rule::TypeId, RuleRegistry::QueryResult::DiscreteValues>& RuleRegistry::DiscreteValueRuleTypes() const {
  return discrete_rule_types_;
}

std::optional<RuleRegistry::QueryResult> RuleRegistry::GetPossibleStatesOfRuleType(const Rule::TypeId& type_id) const {
  const auto range_value_rule_type_it = range_rule_types_.find(type_id);
  if (range_value_rule_type_it != range_rule_types_.end()) {
    return {RuleRegistry::QueryResult{range_value_rule_type_it->first /* type_id */,
                                      range_value_rule_type_it->second /* rule_values */}};
  }

  const auto discrete_value_rule_type_it = discrete_rule_types_.find(type_id);
  if (discrete_value_rule_type_it != discrete_rule_types_.end()) {
    return {RuleRegistry::QueryResult{discrete_value_rule_type_it->first /* type_id */,
                                      discrete_value_rule_type_it->second /* rule_values*/}};
  }

  return std::nullopt;
}

RangeValueRule RuleRegistry::BuildRangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id,
                                                 const LaneSRoute& zone,
                                                 const RuleRegistry::QueryResult::Ranges& ranges) const {
  const auto range_rule_type = range_rule_types_.find(type_id);
  MALIPUT_THROW_UNLESS(range_rule_type != range_rule_types_.end());
  for (const RangeValueRule::Range& range : ranges) {
    MALIPUT_THROW_UNLESS(HasValue(range_rule_type->second, range));
  }

  return RangeValueRule(id, type_id, zone, ranges);
}

DiscreteValueRule RuleRegistry::BuildDiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id,
                                                       const LaneSRoute& zone,
                                                       const RuleRegistry::QueryResult::DiscreteValues& values) const {
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
