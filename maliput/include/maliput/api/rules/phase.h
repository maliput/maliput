// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <optional>
#include <unordered_map>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {
namespace api {
namespace rules {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// A mapping from a RightOfWayRule::Id to a RightOfWayRule::State::Id. Just an
/// alias for user convenience.
using RuleStates MALIPUT_DEPRECATED("RightOfWayRule class will be deprecated.") =
    std::unordered_map<RightOfWayRule::Id, RightOfWayRule::State::Id>;
#pragma GCC diagnostic pop

/// A mapping from a UniqueBulbId to a BulbState. Just an alias for user
/// convenience.
using BulbStates = std::unordered_map<UniqueBulbId, BulbState>;

/// A mapping from a Rule::Id to a DiscreteValueRule::DiscreteValue.
/// Just an alias for user convenience.
using DiscreteValueRuleStates = std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>;

/// A group of RightOfWayRule instances and their states. It models coupling
/// between these rules due to, for example, spatial co-location at
/// intersections.
class Phase final {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Phase);

  /// Unique identifier for a Phase.
  using Id = TypeSpecificIdentifier<Phase>;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// Constructs a Phase.
  ///
  /// @param id the unique ID of the phase (in the RightOfWayRulePhaseRing)
  ///
  /// @param rule_states the RightOfWayRules and their states when the phase is
  /// applied, e.g., to an intersection.
  ///
  /// @param discrete_value_rule_states the DiscreteValueRules and their states
  /// when the phase is applied, e.g., to an intersection. Referenced rule type
  /// must be RightOfWayRuleType().
  ///
  /// @param bulb_states the states of the bulbs when this phase is applied,
  /// e.g., to an intersection.
  ///
  /// @note @p rules_states and @p discrete_value_rule_states should reflect the
  /// same information. Rules should be duplicated while the transition from one
  /// type to the other happens.
  Phase(const Id& id, const RuleStates& rule_states, const DiscreteValueRuleStates& discrete_value_rule_states,
        std::optional<BulbStates> bulb_states = std::nullopt);
#pragma GCC diagnostic pop

  /// Returns the phase's identifier.
  const Id& id() const { return id_; }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// Returns the phase's RightOfWayRule instances and their states.
  MALIPUT_DEPRECATED("RightOfWayRule class will be deprecated.")
  const RuleStates& rule_states() const { return rule_states_; }
#pragma GCC diagnostic pop

  /// Returns the phase's DiscreteValueRule instances and their states.
  const DiscreteValueRuleStates& discrete_value_rule_states() const { return discrete_value_rule_states_; }

  /// Returns the phase's bulb states.
  const std::optional<BulbStates>& bulb_states() const { return bulb_states_; }

 private:
  Id id_;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  RuleStates rule_states_;
#pragma GCC diagnostic pop
  DiscreteValueRuleStates discrete_value_rule_states_;
  std::optional<BulbStates> bulb_states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
