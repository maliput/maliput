#include "maliput/base/manual_right_of_way_rule_state_provider.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_throw.h"

namespace maliput {

using api::rules::RightOfWayRule;

void ManualRightOfWayRuleStateProvider::AddState(const RightOfWayRule::Id& id,
                                                 const RightOfWayRule::State::Id& initial_state) {
  auto result = states_.emplace(id, initial_state);
  if (!result.second) {
    throw std::logic_error("Attempted to add multiple rules with id " + id.string());
  }
}

void ManualRightOfWayRuleStateProvider::SetState(const RightOfWayRule::Id& id, const RightOfWayRule::State::Id& state) {
  states_.at(id) = state;
}

drake::optional<api::rules::RightOfWayRuleStateProvider::RightOfWayResult>
ManualRightOfWayRuleStateProvider::DoGetState(const RightOfWayRule::Id& id) const {
  auto it = states_.find(id);
  if (it == states_.end()) {
    return drake::nullopt;
  }
  return api::rules::RightOfWayRuleStateProvider::RightOfWayResult{it->second, drake::nullopt};
}

}  // namespace maliput
