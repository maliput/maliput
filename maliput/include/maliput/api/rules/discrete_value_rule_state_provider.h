#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the state provider of DiscreteValueRules.
class DiscreteValueRuleStateProvider {
 public:
  /// The state of a DiscreteValueRule, returned by
  /// DiscreteValueRuleStateProvider::GetState(const Rule::Id&).
  struct StateResult {
    /// Information about the next state.
    struct Next {
      DiscreteValueRule::DiscreteValue value_state;
      /// If known, the estimated time until the transition to the next state,
      /// relative to when DiscreteValueRuleStateProvider::GetState() was
      /// called. Users should treat this as advisory since it is tentative and
      /// subject to change at any point in time.
      drake::optional<double> duration_until;
    };

    /// The rule's current state.
    DiscreteValueRule::DiscreteValue value_state;
    /// The rule's next state, if known. Users should treat this as advisory
    /// since it is tentative and subject to change at any point in time.
    drake::optional<Next> next;
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteValueRuleStateProvider)

  virtual ~DiscreteValueRuleStateProvider() = default;

  /// Gets the state of the DiscreteValueRule identified by `id`.
  ///
  /// Returns a StateResult struct bearing the state of the rule with the
  /// specified `id`. If `id` is unrecognized, drake::nullopt is returned.
  drake::optional<StateResult> GetState(const Rule::Id& id) const { return DoGetState(id); }

 protected:
  DiscreteValueRuleStateProvider() = default;

 private:
  virtual drake::optional<StateResult> DoGetState(const Rule::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
