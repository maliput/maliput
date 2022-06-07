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

#include <map>
#include <utility>
#include <variant>
#include <vector>

#include "maliput/api/regions.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace rules {

/// Convenient alias for clients that define DiscreteValueRule types.
using DiscreteValueRuleTypeAndValues = std::pair<Rule::TypeId, std::vector<DiscreteValueRule::DiscreteValue>>;

/// Convenient alias for clients that define RangeValueRule types.
using RangeValueRuleTypeAndValues = std::pair<Rule::TypeId, std::vector<RangeValueRule::Range>>;

/// A registry for Rule types.
///
/// A Rule type is distinguished by its Rule::TypeId, which must be unique among
/// all Rules (including both RangeValueRules and DiscreteValueRules). This
/// class provides a registry of the various rule types, and enables semantic
/// validation when building rule instances.
class RuleRegistry {
 public:
  /// Holds a rule type information for a query.
  struct QueryResult {
    using Ranges = std::vector<RangeValueRule::Range>;
    using DiscreteValues = std::vector<DiscreteValueRule::DiscreteValue>;

    Rule::TypeId type_id;
    /// Holds either `Ranges` or `DiscreteValues` for the rule type.
    std::variant<Ranges, DiscreteValues> rule_values;
  };

  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RuleRegistry);
  RuleRegistry() = default;
  virtual ~RuleRegistry() = default;

  /// Registers a RangeValueRule type.
  ///
  /// @param type_id The Rule type.
  /// @param all_possible_ranges All possible ranges that a rule of
  ///        this type could be in. It must have at least one value; each value
  ///        must be unique.
  /// @see RangeValueRule
  /// @throws maliput::common::assertion_error When `type_id` is already
  ///         registered.
  /// @throws maliput::common::assertion_error When `all_possible_ranges` is
  ///         empty.
  /// @throws maliput::common::assertion_error When there are duplicated items
  ///         in `all_possible_ranges`.
  void RegisterRangeValueRule(const Rule::TypeId& type_id,
                              const std::vector<RangeValueRule::Range>& all_possible_ranges);

  /// Registers a DiscreteValueRule type.
  ///
  /// @param type_id The Rule type.
  /// @param all_possible_values All possible discrete values that a rule of
  ///        this type could be in. It must have at least one value; each value
  ///        must be unique.
  /// @see DiscreteValueRule.
  /// @throws maliput::common::assertion_error When `type_id` is already
  ///         registered.
  /// @throws maliput::common::assertion_error When `all_possible_values` is
  ///         empty.
  /// @throws maliput::common::assertion_error When there are duplicated items
  ///         in `all_possible_values`.
  void RegisterDiscreteValueRule(const Rule::TypeId& type_id,
                                 const std::vector<DiscreteValueRule::DiscreteValue>& all_possible_values);

  /// @returns All of the registered RangeValueRule types and their possible
  ///          ranges.
  const std::map<Rule::TypeId, std::vector<RangeValueRule::Range>>& RangeValueRuleTypes() const;

  /// @returns All of the registered DiscreteValueRule types and their possible
  ///          values.
  const std::map<Rule::TypeId, std::vector<DiscreteValueRule::DiscreteValue>>& DiscreteValueRuleTypes() const;

  /// Finds the possible states of a rule type by `type_id`.
  ///
  /// @param type_id Rule type ID.
  /// @returns The possible states of a specified rule type, or nullopt if
  ///          no such rule type exists.
  std::optional<QueryResult> GetPossibleStatesOfRuleType(const Rule::TypeId& type_id) const;

  /// Builds a RangeValueRule.
  ///
  /// @see RangeValueRule constructor for parameter documentation.
  /// @throws maliput::common::assertion_error When `type_id` is not a
  ///         registered RangeValueRule type.
  /// @throws maliput::common::assertion_error When an element in `ranges` is
  ///         not a possible range of a RangeValueRule of type `type_id`.
  RangeValueRule BuildRangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                                     const std::vector<RangeValueRule::Range>& ranges) const;

  /// Builds a DiscreteValueRule.
  ///
  /// @see DiscreteValueRule constructor for parameter documentation.
  /// @throws maliput::common::assertion_error When `type_id` is not a
  ///         registered DiscreteValueRule type.
  /// @throws maliput::common::assertion_error When an element in `values` is
  ///         not a possible value of a DiscreteValueRule of type `type_id`.
  DiscreteValueRule BuildDiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                                           const std::vector<DiscreteValueRule::DiscreteValue>& values) const;

 private:
  std::map<Rule::TypeId, std::vector<RangeValueRule::Range>> range_rule_types_;
  std::map<Rule::TypeId, std::vector<DiscreteValueRule::DiscreteValue>> discrete_rule_types_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
