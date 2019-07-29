#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the provider of RangeValueRule.
class RangeValueRuleStateProvider {
 public:
  /// Result returned by GetState(const Rule::Id).
  struct RangeValueResult {
    /// Information about a subsequent state.
    struct Next {
      /// State value.
      RangeValueRule::Range state_range;
      /// If known, estimated time until the transition to the state.
      drake::optional<double> duration_until;
    };

    /// ID of the rule's current state value.
    RangeValueRule::Range state_range;
    /// Information about the rule's upcoming state if a state transition is
    /// anticipated.
    drake::optional<Next> next;
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RangeValueRuleStateProvider)

  virtual ~RangeValueRuleStateProvider() = default;

  /// Gets the state of the RangeValueRule identified by `id`.
  ///
  /// Returns a RangeValueResult struct bearing the RangeValueRule::Range state
  /// of the rule's current state. If a transition to a new state is
  /// anticipated, RangeValueResult::next will be populated and bear the
  /// RangeValueRule::Range of the next state. If the time until the transition
  /// is known, then RangeValueResult::next.duration_until will be populated
  /// with that duration.
  ///
  /// Returns drake::nullopt if `id` is unrecognized.
  drake::optional<RangeValueResult> GetState(const Rule::Id& id) const { return DoGetState(id); }

 protected:
  RangeValueRuleStateProvider() = default;

 private:
  virtual drake::optional<RangeValueResult> DoGetState(const Rule::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
