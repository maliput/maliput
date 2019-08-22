#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/type_specific_identifier.h"

namespace maliput {
namespace api {
namespace rules {

/// A registry for Rule types.
///
/// A Rule type is distinguished by its Rule::TypeId, which must be unique among
/// all Rules (including both RangeValueRules and DiscreteValueRules). This
/// class provides a registry of the various rule types, and enables semantic
/// validation when building rule instances.
class RuleRegistry {
 public:
  /// Wrapper similar to std::variant to return rule type information.
  ///
  /// At most one of the two optional attributes will have a value.
  struct QueryResult {
    Rule::TypeId type_id;

    /// Defines the possible ranges of a range value rule type.
    drake::optional<std::vector<RangeValueRule::Range>> range_values;

    /// Defines the possible discrete values of a discrete value rule type.
    drake::optional<std::vector<std::string>> discrete_values;
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RuleRegistry);
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
  void RegisterDiscreteValueRule(const Rule::TypeId& type_id, const std::vector<std::string>& all_possible_values);

  /// @returns All of the registered RangeValueRule types and their possible
  ///          ranges.
  const std::map<Rule::TypeId, std::vector<RangeValueRule::Range>>& RangeValueRuleTypes() const;

  /// @returns All of the registered DiscreteValueRule types and their possible
  ///          values.
  const std::map<Rule::TypeId, std::vector<std::string>>& DiscreteValueRuleTypes() const;

  /// Finds the possible states of a rule type by `type_id`.
  ///
  /// @param type_id Rule type ID.
  /// @returns The possible states of a specified rule type, or nullopt if
  ///          no such rule type exists.
  drake::optional<QueryResult> GetPossibleStatesOfRuleType(const Rule::TypeId& type_id) const;

  /// Builds a RangeValueRule.
  ///
  /// @see RangeValueRule constructor for parameter documentation.
  /// @throws maliput::common::assertion_error When `type_id` is not a
  ///         registered RangeValueRule type.
  /// @throws maliput::common::assertion_error When an element in `ranges` is
  ///         not a possible range of a RangeValueRule of type `type_id`.
  RangeValueRule BuildRangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                                     const std::vector<Rule::Id>& related_rules,
                                     const std::vector<RangeValueRule::Range>& ranges) const;

  /// Builds a DiscreteValueRule.
  ///
  /// @see DiscreteValueRule constructor for parameter documentation.
  /// @throws maliput::common::assertion_error When `type_id` is not a
  ///         registered DiscreteValueRule type.
  /// @throws maliput::common::assertion_error When an element in `values` is
  ///         not a possible value of a DiscreteValueRule of type `type_id`.
  DiscreteValueRule BuildDiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                                           const std::vector<Rule::Id>& related_rules,
                                           const std::vector<std::string>& values) const;

 private:
  std::map<Rule::TypeId, std::vector<RangeValueRule::Range>> range_rule_types_;
  std::map<Rule::TypeId, std::vector<std::string>> discrete_rule_types_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
