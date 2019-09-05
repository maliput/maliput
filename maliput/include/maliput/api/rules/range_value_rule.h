#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"

#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

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
                 const std::vector<Rule::Id> related_rules, const std::vector<Range>& ranges);

  const std::vector<Range>& ranges() const { return ranges_; }

 private:
  std::vector<Range> ranges_;
};

/// Constructs a RangeValueRule::RangeValue.
// TODO(maliput #121) Remove this once we switch to C++17 and can use aggregate initialization.
RangeValueRule::Range MakeRange(int severity, const std::string& description, double min, double max);

}  // namespace rules
}  // namespace api
}  // namespace maliput
