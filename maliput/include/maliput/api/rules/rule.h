#pragma once

#include <functional>
#include <set>
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
/// A Rule may have multiple states that affect agent behavior in different ways
/// while driving through the rule's zone. The possible states of a Rule must be
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
  /// each Rule's instance.
  /// Backend implementations should consider the standard
  /// "[rule_type]/[rule_id]" for rule naming.
  using Id = TypeSpecificIdentifier<class Rule>;

  /// Alias for the Rule's type. Several instances could share this property.
  using TypeId = TypeSpecificIdentifier<class Type>;

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rule);

  /// Constructs a Rule.
  ///
  /// @param id The Rule ID.
  /// @param type_id The Rule Type ID.
  /// @param zone LaneSRoute to which this rule applies.
  /// @param related_rules A vector of related rules.
  Rule(const Id& id, const TypeId& type_id, const LaneSRoute& zone,
       const std::vector<Id>& related_rules) :
      id_(id), type_id_(type_id), zone_(zone), related_rules_(related_rules) {}

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

/// Describes a numeric range based constraint.
///
/// Ranges are closed and continuous, defined by a minimum and maximum quantity.
/// When only one of the extremes is defined, the other should take a
/// semantically correct value. For example, a speed limit rule may only have a
/// positive maximum value defined, then most probably a correct minimum would
/// be 0 to avoid traveling in -s direction.
///
/// Magnitudes are considered to be described in SI by default.
class RangeValueRule : public Rule {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RangeValueRule);

  /// Defines a range for the rule.
  struct Range {
    /// Convenient operator overload to order Ranges in std ordered collections.
    bool operator<(const Range& other) const {
      if (min < other.min)        return true;
      if (min > other.min)        return false;
      if (max < other.max)        return true;
      if (max < other.max)        return false;
      return (description < other.description);
    }

    std::string description;  ///< Semantics of the range quantity.
    double min{};             ///< Minimum value of the range.
    double max{};             ///< Maximum value of the range.
  };

  /// Constructs a range based rule.
  ///
  /// @param id The Rule ID.
  /// @param type_id The Rule Type ID.
  /// @param zone LaneSRoute to which this rule applies.
  /// @param related_rules A vector of related rules.
  /// @param ranges A set of ranges. It must have at least one item and all
  ///        of them should respect that min <= max.
  /// @throws maliput::common::assertion_error When `ranges` is empty.
  /// @throws maliput::common::assertion_error When any Range within `ranges`
  ///         violates `min <= max` condition.
  RangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id,
                 const LaneSRoute& zone,
                 const std::vector<Rule::Id> related_rules,
                 const std::set<Range>& ranges) :
      Rule(id, type_id, zone, related_rules), ranges_(ranges) {
    MALIPUT_THROW_UNLESS(!ranges_.empty());
    for (const Range& range : ranges_) {
      MALIPUT_THROW_UNLESS(range.min <= range.max);
    }
  }

  const std::set<Range> ranges() const { return ranges_; }

 private:
  std::set<Range> ranges_;
};

/// Describes an arbitrary discrete and string based rule.
///
/// Semantics of this rule are provided by _all_ possible values that this
/// Rule::TypeId could have, not only the ones that an specific instance of
/// this rule provides.
class DiscreteValueRule : public Rule {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteValueRule);

  /// Constructs a discrete value based rule.
  ///
  /// @param id The Rule ID.
  /// @param type_id The Rule Type ID.
  /// @param zone LaneSRoute to which this rule applies.
  /// @param related_rules A vector of related rules.
  /// @param ranges A set of value states. It must have at least one item.
  /// @throws maliput::common::assertion_error When `value_states` is empty.
  DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id,
                    const LaneSRoute& zone,
                    const std::vector<Id> related_rules,
                    const std::set<std::string>& value_states) :
      Rule(id, type_id, zone, related_rules), value_states_(value_states) {
    MALIPUT_THROW_UNLESS(!value_states_.empty());
  }

  const std::set<std::string> value_states() const { return value_states_; }

 private:
  std::set<std::string> value_states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
