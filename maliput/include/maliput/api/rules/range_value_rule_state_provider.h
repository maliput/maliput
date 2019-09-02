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
  struct StateResult {
    /// Information about a subsequent state.
    struct Next {
      /// State value.
      RangeValueRule::Range state_range;
      /// If known, estimated time until the transition to the state, relative
      /// to when RangeValueRuleStateProvider::GetState() was called.
      drake::optional<double> duration_until;
    };

    /// The rule's current state.
    RangeValueRule::Range state_range;
    /// The rule's next state, if known.
    drake::optional<Next> next;
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RangeValueRuleStateProvider)

  virtual ~RangeValueRuleStateProvider() = default;

  /// Gets the state of the RangeValueRule identified by `id`.
  ///
  /// Returns a StateResult struct bearing the state of the rule's
  /// current state identified by `id`. When `id` is unrecognized,
  /// drake::nullopt is returned.
  drake::optional<StateResult> GetState(const Rule::Id& id) const { return DoGetState(id); }

 protected:
  RangeValueRuleStateProvider() = default;

 private:
  virtual drake::optional<StateResult> DoGetState(const Rule::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
