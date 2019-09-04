#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"

#include "maliput/api/rules/regions.h"
#include "maliput/api/type_specific_identifier.h"
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

  /// Defines a base state for a Rule.
  struct State {
    /// Defines common Rule severity levels. Specific rule types can choose to use
    /// these, or define their own custom levels.
    ///@{

    /// Rule must always be obeyed.
    static constexpr int kStrict{0};

    /// Rule should be obeyed on a best-effort basis.
    static constexpr int kBestEffort{1};

    ///@}

    bool operator==(const State& other) const { return severity == other.severity; }
    bool operator!=(const State& other) const { return !(*this == other); }

    /// Severity of the Rule::State. A non-negative quantity that specifies the
    /// level of enforcement. The smaller it is, the more strictly the rule is
    /// enforced. Each rule type can define its own set of severity level
    /// semantics. See kStrict and kBestEffort for two commonly used severity
    /// levels.
    int severity{};
  };

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
  Rule(const Id& id, const TypeId& type_id, const LaneSRoute& zone, const std::vector<Id>& related_rules)
      : id_(id), type_id_(type_id), zone_(zone), related_rules_(related_rules) {
    for (const Rule::Id& rule_id : related_rules_) {
      MALIPUT_THROW_UNLESS(std::count(related_rules_.begin(), related_rules_.end(), rule_id) == 1);
    }
  }

  virtual ~Rule() = default;

  const Id& id() const { return id_; }

  const TypeId& type_id() const { return type_id_; }

  const LaneSRoute& zone() const { return zone_; }

  const std::vector<Id>& related_rules() const { return related_rules_; }

 protected:
  // Validates that `severity` is a non-negative quantity.
  // @throws maliput::assertion_error When `severity` is negative.
  void ValidateSeverity(int severity) { MALIPUT_THROW_UNLESS(severity >= 0); }

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
  struct Range : public Rule::State {
    bool operator==(const Range& other) const {
      return min == other.min && max == other.max && description == other.description && Rule::State::operator==(other);
    }
    bool operator!=(const Range& other) const { return !(*this == other); }

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
  RangeValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                 const std::vector<Rule::Id> related_rules, const std::vector<Range>& ranges)
      : Rule(id, type_id, zone, related_rules), ranges_(ranges) {
    MALIPUT_THROW_UNLESS(!ranges_.empty());
    for (const Range& range : ranges_) {
      ValidateSeverity(range.severity);
      MALIPUT_THROW_UNLESS(range.min <= range.max);
      MALIPUT_THROW_UNLESS(std::count(ranges_.begin(), ranges_.end(), range) == 1);
    }
  }

  const std::vector<Range>& ranges() const { return ranges_; }

 private:
  std::vector<Range> ranges_;
};

/// Constructs a RangeValueRule::RangeValue.
// TODO(maliput #121) Remove this once we switch to C++17 and can use aggregate initialization.
inline RangeValueRule::Range MakeRange(int severity, const std::string& description, double min, double max) {
  RangeValueRule::Range range;
  range.severity = severity;
  range.description = description;
  range.min = min;
  range.max = max;
  return range;
}

/// Describes a discrete value rule.
///
/// DiscreteValues are defined by a string value.
/// Semantics of this rule are based on _all_ possible values that this
/// Rule::TypeId could have (as specified by RuleRegistry::FindRuleByType()),
/// not only the subset of values that a specific instance of this rule can
/// be in.
class DiscreteValueRule : public Rule {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiscreteValueRule);

  /// Defines a discrete value for a DiscreteValueRule.
  struct DiscreteValue : public Rule::State {
    bool operator==(const DiscreteValue& other) const { return value == other.value && Rule::State::operator==(other); }
    bool operator!=(const DiscreteValue& other) const { return !(*this == other); }

    std::string value;  ///< Value of the DiscreteValue.
  };

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
  DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                    const std::vector<Id> related_rules, const std::vector<DiscreteValue>& values)
      : Rule(id, type_id, zone, related_rules), values_(values) {
    MALIPUT_THROW_UNLESS(!values_.empty());
    for (const DiscreteValue& value : values_) {
      ValidateSeverity(value.severity);
      MALIPUT_THROW_UNLESS(std::count(values_.begin(), values_.end(), value) == 1);
    }
  }

  const std::vector<DiscreteValue>& values() const { return values_; }

 private:
  std::vector<DiscreteValue> values_;
};

/// Constructs a DiscreteValueRule::DiscreteValue.
// TODO(maliput #121) Remove this once we switch to C++17 and can use aggregate initialization.
inline DiscreteValueRule::DiscreteValue MakeDiscreteValue(int severity, const std::string& value) {
  DiscreteValueRule::DiscreteValue discrete_value;
  discrete_value.severity = severity;
  discrete_value.value = value;
  return discrete_value;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
