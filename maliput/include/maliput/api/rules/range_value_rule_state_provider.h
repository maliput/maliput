#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the state provider of RangeValueRules.
class RangeValueRuleStateProvider {
 public:
  /// The state of a RangeValueRule, returned by
  /// RangeValueRuleStateProvider::GetState(const Rule::Id&).
  struct StateResult {
    /// Information about the next state.
    struct Next {
      RangeValueRule::Range range_state;
      /// If known, the estimated time until the transition to the next state,
      /// relative to when RangeValueRuleStateProvider::GetState() was
      /// called.
      drake::optional<double> duration_until;
    };

    /// The rule's current state.
    RangeValueRule::Range range_state;
    /// The rule's next state, if known.
    drake::optional<Next> next;
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RangeValueRuleStateProvider)

  virtual ~RangeValueRuleStateProvider() = default;

  /// Gets the state of the RangeValueRule identified by `id`.
  ///
  /// Returns a StateResult struct bearing the state of the rule with the
  /// specified `id`. If `id` is unrecognized, drake::nullopt is returned.
  drake::optional<StateResult> GetState(const Rule::Id& id) const { return DoGetState(id); }

 protected:
  RangeValueRuleStateProvider() = default;

 private:
  virtual drake::optional<StateResult> DoGetState(const Rule::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
