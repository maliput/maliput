#pragma once

#include <map>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/type_specific_identifier.h"
#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

/// Rule registry for handling Rule types.
///
/// Rules could have multiple states, and be of multiple types but semantic
/// meaning of a Rule is not hard typed except of using DiscreteValueRule or
/// RangeValueRule. This class provides a convenient registry of the available
/// rule types, and allows semantic validation when building rule instances.
///
/// Rule type IDs are enforced to be unique among different rule types,
/// including both RangeValueRules and DiscreteValueRules.
class RuleRegistry {
 public:
  /// Wrapper similar to std::variant to return rule type information.
  ///
  /// No more than one of the optional attributes must be assigned at the same
  /// time.
  struct QueryResult {
    /// Defines a range value rule type.
    drake::optional<Rule::TypeId> range_value_rule_type{};
    /// Defines a discrete value rule type.
    drake::optional<std::pair<Rule::TypeId, std::vector<std::string>>>
        discrete_value_rule_type{};
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RuleRegistry);
  RuleRegistry() = default;
  virtual ~RuleRegistry() = default;

  /// Registers a RangeValueRule type.
  ///
  /// @param type_id The Rule type.
  /// @see RangeValueRule
  /// @throws maliput::common::assertion_error When `type_id` is already
  ///         registered.
  void RegisterRangeValueRule(const Rule::TypeId& type_id);

  /// Registers a DiscreteValueRule type.
  ///
  /// @param type_id The Rule type.
  /// @param all_possible_values A vector of all possible discrete values that
  ///        a rule could be in. It must have at least one value; each value
  ///        must be unique.
  /// @see DiscreteValueRule.
  /// @throws maliput::common::assertion_error When `type_id` is already
  ///         registered.
  /// @throws maliput::common::assertion_error When `all_possible_valuess`
  ///         is empty.
  /// @throws maliput::common::assertion_error When there are duplicated items
  ///         in `all_possible_values`.
  void RegisterDiscreteValueRule(
      const Rule::TypeId& type_id,
      const std::vector<std::string>& all_possible_values);

  /// @returns A vector of Rule::TypeId with all the available range based rule
  ///          types.
  const std::vector<Rule::TypeId>& RangeValueRuleTypes() const;

  /// @returns A map of Rule::TypeId to vector of strings with all the available
  ///          discrete value based rule types.
  const std::map<Rule::TypeId, std::vector<std::string>>&
      DiscreteValueRuleTypes() const;

  /// Finds a rule type by `type_id`.
  ///
  /// @param type_id Rule type ID.
  /// @returns A QueryResult with all elements set to nullopt when no rule was
  ///          found. Otherwise, the found rule type will be loaded.
  QueryResult FindRuleTypeBy(const Rule::TypeId& type_id) const;

  /// Builds a RangeValueRule whose `type_id` is registered.
  ///
  /// @see RangeValueRule constructor for parameter documentation.
  /// @throws maliput::common::assertion_error When `type_id` is not a
  ///         registered range value rule type.
  RangeValueRule BuildRangeValueRule(
      const Rule::Id& id, const Rule::TypeId& type_id,
      const LaneSRoute& zone, const std::vector<Rule::Id>& related_rules,
      const std::vector<RangeValueRule::Range>& ranges) const;

  /// Builds a DiscreteValueRule whose `type_id` is registered.
  ///
  /// @see DiscreteValueRule constructor for parameter documentation.
  /// @throws maliput::common::assertion_error When `type_id` is not a
  ///         registered discrete value rule type.
  /// @throws maliput::common::assertion_error When any value in `values` is not
  ///         a registered discrete value in `type_id`.
  DiscreteValueRule BuildDiscreteValueRule(
      const Rule::Id& id, const Rule::TypeId& type_id,
      const LaneSRoute& zone, const std::vector<Rule::Id>& related_rules,
      const std::vector<std::string>& values) const;

 private:
  std::vector<Rule::TypeId> range_rule_types_;
  std::map<Rule::TypeId, std::vector<std::string>> discrete_rule_types_;
};


}  // namespace rules
}  // namespace api
}  // namespace maliput
