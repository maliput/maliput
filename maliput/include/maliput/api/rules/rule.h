#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"

#include "maliput/api/type_specific_identifier.h"
#include "maliput/api/rules/regions.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

/// Describes a generic rule type.
///
/// A Rule may have multiple states that affect agent behavior while it is
/// driving through the rule's zone. The possible states of a Rule must be
/// semantically coherent. The current state of a Rule is given by a
/// RuleStateProvider. States can be:
///
/// - range based (@see RangeValueRule).
/// - discrete (@see DiscreteValueRule).
///
/// Finally, a Rule may point to other Rules via `related_rules`.
class Rule {
 public:

  /// Used to specialize TypeSpecificIdentifier and define a unique TypeId.
  class Type;

  /// Alias for the Rule's unique ID across all Rule types. It is a property of
  /// each Rule's instance, and should be unique across all Rule instances.
  /// To achieve this, backend implementations are encouraged to use
  /// "[rule_type]/[rule_id]" as the string value of a Rule's ID.
  using Id = TypeSpecificIdentifier<class Rule>;

  /// Alias for the Rule's type. Several Rule instances could share the same
  /// TypeId, assuming they really are the same type. Example types include
  /// "right of way rule", "direction usage rule", "vehicle usage rule", etc.
  using TypeId = TypeSpecificIdentifier<class Type>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rule);

  /// Constructs a Rule.
  ///
  /// @param id The Rule ID.
  /// @param type_id The Rule Type ID.
  /// @param zone LaneSRoute to which this rule applies.
  /// @param related_rules A vector of related rules. The semantics vary based
  ///        on the specific rule type. Each ID must be unique.
  ///
  /// @throws maliput::common::assertion_error When any Rule::Id within
  ///         `related_rules` is duplicated.
  Rule(const Id& id, const TypeId& type_id, const LaneSRoute& zone,
       const std::vector<Id>& related_rules) :
      id_(id), type_id_(type_id), zone_(zone), related_rules_(related_rules) {
    for (const Rule::Id& rule_id : related_rules_) {
      MALIPUT_THROW_UNLESS(
          std::count(related_rules_.begin(), related_rules_.end(), rule_id) ==
          1);
    }
  }

  virtual ~Rule() = default;

  const Id& id() const { return id_; }

  const TypeId& type_id() const { return type_id_; }

  const LaneSRoute& zone() const { return zone_; }

  const std::vector<Id>& related_rules() const { return related_rules_; }

 private:
  Id id_;
  TypeId type_id_;
  LaneSRoute zone_;
  std::vector<Id> related_rules_;
};

/// Describes a numeric range based rule.
///
/// Ranges are closed and continuous, defined by a minimum and maximum quantity.
/// When only one extreme is formally defined, the other should take a
/// semantically correct value. For example, if a speed limit only specifies a
/// maximum value, the minimum value is typically zero.
class RangeValueRule : public Rule {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RangeValueRule);

  /// Defines a range for a RangeValueRule.
  struct Range {
    bool operator==(const Range& other) const {
      return min == other.min && max == other.max &&
             description == other.description;
    }

    bool operator!=(const Range& other) const {
      return !(*this == other);
    }

    std::string description;  ///< Semantics of the range quantity.
    double min{};             ///< Minimum value of the range.
    double max{};             ///< Maximum value of the range.
  };

  /// Constructs a range based Rule.
  ///
  /// @param id The Rule ID.
  /// @param type_id The Rule Type ID.
  /// @param zone LaneSRoute to which this rule applies.
  /// @param related_rules A vector of related rules.
  /// @param ranges A vector of possible ranges that this rule could enforce.
  ///               The actual range that's enforced at any given time is
  ///               determined by a RangeValueRuleStateProvider. This vector
  ///               must have at least one Range, and each Range must respect
  ///               that its min <= max and be unique.
  /// @throws maliput::common::assertion_error When `ranges` is empty.
  /// @throws maliput::common::assertion_error When any Range within `ranges`
  ///         violates `min <= max` condition.
  /// @throws maliput::common::assertion_error When there are duplicated Range
  ///         in `ranges`.
  RangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id,
                 const LaneSRoute& zone,
                 const std::vector<Rule::Id> related_rules,
                 const std::vector<Range>& ranges) :
      Rule(id, type_id, zone, related_rules), ranges_(ranges) {
    MALIPUT_THROW_UNLESS(!ranges_.empty());
    for (const Range& range : ranges_) {
      MALIPUT_THROW_UNLESS(range.min <= range.max);
      MALIPUT_THROW_UNLESS(std::count(ranges_.begin(), ranges_.end(), range) ==
                           1);
    }
  }

  const std::vector<Range>& ranges() const { return ranges_; }

 private:
  std::vector<Range> ranges_;
};

/// Describes an arbitrary discrete and string based rule.
///
/// Semantics of this rule are provided by _all_ possible values that this
/// Rule::TypeId could have, not only the ones that a specific instance of
/// this rule provides.
class DiscreteValueRule : public Rule {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteValueRule);

  /// Constructs a DiscreteValueRule.
  ///
  /// @param id The Rule ID.
  /// @param type_id The Rule Type ID.
  /// @param zone LaneSRoute to which this rule applies.
  /// @param related_rules A vector of related rules.
  /// @param values A vector of possible discrete values that this Rule could be
  ///               in. The actual value that's enforced at any given time is
  ///               determined by a DiscreteValueRuleStateProvider.  It must
  ///               have at least one value; each value must be unique.
  /// @throws maliput::common::assertion_error When `values` is empty.
  /// @throws maliput::common::assertion_error When there are duplicated values
  ///         in `values`.
  DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id,
                    const LaneSRoute& zone,
                    const std::vector<Id> related_rules,
                    const std::vector<std::string>& values) :
      Rule(id, type_id, zone, related_rules), values_(values) {
    MALIPUT_THROW_UNLESS(!values_.empty());
    for (const std::string& value : values_) {
      MALIPUT_THROW_UNLESS(
          std::count(values_.begin(), values_.end(), value) == 1);
    }
  }

  const std::vector<std::string>& values() const { return values_; }

 private:
  std::vector<std::string> values_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
