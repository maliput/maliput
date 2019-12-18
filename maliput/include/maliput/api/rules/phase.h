#pragma once

#include <unordered_map>
#include <optional>

#include "drake/common/drake_copyable.h"

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/api/type_specific_identifier.h"

namespace maliput {
namespace api {
namespace rules {

/// A mapping from a RightOfWayRule::Id to a RightOfWayRule::State::Id. Just an
/// alias for user convenience.
using RuleStates = std::unordered_map<RightOfWayRule::Id, RightOfWayRule::State::Id>;

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Phase);

  /// Unique identifier for a Phase.
  using Id = TypeSpecificIdentifier<Phase>;

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

  /// Returns the phase's identifier.
  const Id& id() const { return id_; }

  /// Returns the phase's RightOfWayRule instances and their states.
  const RuleStates& rule_states() const { return rule_states_; }

  /// Returns the phase's DiscreteValueRule instances and their states.
  const DiscreteValueRuleStates& discrete_value_rule_states() const { return discrete_value_rule_states_; }

  /// Returns the phase's bulb states.
  const std::optional<BulbStates>& bulb_states() const { return bulb_states_; }

 private:
  Id id_;
  RuleStates rule_states_;
  DiscreteValueRuleStates discrete_value_rule_states_;
  std::optional<BulbStates> bulb_states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
