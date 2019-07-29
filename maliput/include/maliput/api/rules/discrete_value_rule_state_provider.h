#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the provider of DiscreteValueRule.
class DiscreteValueRuleStateProvider {
 public:
  /// Result returned by GetState(const Rule::Id).
  struct DiscreteValueResult {
    /// Information about a subsequent state.
    struct Next {
      /// State value.
      std::string state_value;
      /// If known, estimated time until the transition to the state.
      drake::optional<double> duration_until;
    };

    /// ID of the rule's current state value.
    std::string state_value;
    /// Information about the rule's upcoming state if a state transition is
    /// anticipated.
    drake::optional<Next> next;
  };

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteValueRuleStateProvider)

  virtual ~DiscreteValueRuleStateProvider() = default;

  /// Gets the state of the DiscreteValueRule identified by `id`.
  ///
  /// Returns a DiscreteValueResult struct bearing the string state of the
  /// rule's current state. If a transition to a new state is anticipated,
  /// DiscreteValueResult::next will be populated and bear the string of the
  /// next state. If the time until the transition is known, then
  /// DiscreteValueResult::next.duration_until will be populated with that
  /// duration.
  ///
  /// Returns drake::nullopt if `id` is unrecognized.
  drake::optional<DiscreteValueResult> GetState(const Rule::Id& id) const { return DoGetState(id); }

 protected:
  DiscreteValueRuleStateProvider() = default;

 private:
  virtual drake::optional<DiscreteValueResult> DoGetState(const Rule::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
