#pragma once

#include <optional>

#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/state_provider_result.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the provider of the RightOfWayRule.
class MALIPUT_DEPRECATED("next release", "RigthOfWayRule class will be deprecated.") RightOfWayRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayRuleStateProvider)

  virtual ~RightOfWayRuleStateProvider() = default;

  /// Result returned by GetState(const RightOfWayRule::Id).
  using RightOfWayResult = StateProviderResult<RightOfWayRule::State::Id>;

  /// Gets the state of the RightOfWayRule identified by `id`.
  ///
  /// Returns a RightOfWayResult struct bearing the State::Id of the rule's
  /// current state.  If a transition to a new state is anticipated,
  /// RightOfWayResult::next will be populated and bear the State::Id of the
  /// next state.  If the time until the transition is known, then
  /// RightOfWayResult::next.duration_until will be populated with that
  /// duration.
  ///
  /// Returns std::nullopt if `id` is unrecognized, which would be the case
  /// if no such rule exists or if the rule has only static semantics.
  std::optional<RightOfWayResult> GetState(const RightOfWayRule::Id& id) const { return DoGetState(id); }

 protected:
  RightOfWayRuleStateProvider() = default;

 private:
  virtual std::optional<RightOfWayResult> DoGetState(const RightOfWayRule::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
