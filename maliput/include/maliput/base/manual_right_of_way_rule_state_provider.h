#pragma once

#include <unordered_map>

#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// A trivial implementation of an api::rules::RightOfWayRuleStateProvider.
class MALIPUT_DEPRECATED("api::rules::RightOfWayRuleStateProvider class will be deprecated.",
                         "ManualDiscreteValueRuleStateProvider") ManualRightOfWayRuleStateProvider final
    : public api::rules::RightOfWayRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ManualRightOfWayRuleStateProvider)

  /// Default constructor.
  ManualRightOfWayRuleStateProvider() {}

  ~ManualRightOfWayRuleStateProvider() final = default;

  /// Adds a dynamic state to this provider.
  ///
  /// @throws std::logic_error if a RightOfWayRule with an ID of @p id
  /// already exists in this provider.
  /// @throws maliput::common::assertion_error if the dynamic state failed to be
  /// added.
  void AddState(const api::rules::RightOfWayRule::Id& id, const api::rules::RightOfWayRule::State::Id& initial_state);

  /// Sets the dynamic state of a RightOfWayRule within this provider.
  ///
  /// @throws std::out_of_range if no dynamic state with @p id exists in this
  /// provider.
  void SetState(const api::rules::RightOfWayRule::Id& id, const api::rules::RightOfWayRule::State::Id& state);

 private:
  std::optional<api::rules::RightOfWayRuleStateProvider::RightOfWayResult> DoGetState(
      const api::rules::RightOfWayRule::Id& id) const final;

  std::unordered_map<api::rules::RightOfWayRule::Id, api::rules::RightOfWayRule::State::Id> states_;
};
#pragma GCC diagnostic pop

}  // namespace maliput
