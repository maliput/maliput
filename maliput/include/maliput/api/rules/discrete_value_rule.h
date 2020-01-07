#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"

#include "maliput/api/regions.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

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
  /// @param values A vector of possible discrete values that this Rule could be
  ///               in. The actual value that's enforced at any given time is
  ///               determined by a DiscreteValueRuleStateProvider.  It must
  ///               have at least one value; each value must be unique.
  /// @throws maliput::common::assertion_error When `values` is empty.
  /// @throws maliput::common::assertion_error When there are duplicated values
  ///         in `values`.
  DiscreteValueRule(const Rule::Id& id, const Rule::TypeId& type_id, const LaneSRoute& zone,
                    const std::vector<DiscreteValue>& values);

  const std::vector<DiscreteValue>& values() const { return values_; }

 private:
  std::vector<DiscreteValue> values_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
