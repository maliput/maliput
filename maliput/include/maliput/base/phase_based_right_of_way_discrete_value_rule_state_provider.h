#pragma once

#include <optional>

#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule.h"
#include "maliput/base/manual_discrete_value_rule_state_provider.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {

/// Provides the state of Right-Of-Way api::rules::DiscreteValueRule instances
/// based on the current api::rules::Phase. The states of the rules that govern
/// an intersection are organized into phases. Each phase typically assigns
/// different states to each rule to ensure intersection safety and fairness.
/// For example, given an intersection between streets A and B governed by
/// Rule_A and Rule_B, respectively, two phases are necessary:
///
/// Phase  | Rule_A State | Rule_B State
/// ------ | ------------ | ------------
///   1    | Go           | Stop
///   2    | Stop         | Go
///
/// The rules above will ensure vehicles traveling on Street A do not collide
/// with vehicles traveling on Street B and vice versa.
///
/// This state provider also acts as a ManualDiscreteValueRuleStateProvider, but
/// it overrides the behavior for Right-Of-Way rules with the aforementioned
/// one. At build time, it is expected that the loader calls
/// ManualDiscreteValueRuleStateProvider::SetState() for those non Right-Of-Way
/// rules that are part of the RoadRulebook.
class PhaseBasedRightOfWayDiscreteValueRuleStateProvider final : public ManualDiscreteValueRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseBasedRightOfWayDiscreteValueRuleStateProvider)

  /// Constructs a PhaseBasedRightOfWayDiscreteValueRuleStateProvider.
  ///
  /// All pointer parameters are aliased; they must not be nullptr and their
  /// lifespans must exceed that of this instance.
  ///
  /// @throws common::assertion_error When `rulebook` is nullptr.
  /// @throws common::assertion_error When `phase_ring_book` is nullptr.
  /// @throws common::assertion_error When `phase_provider` is nullptr.
  PhaseBasedRightOfWayDiscreteValueRuleStateProvider(const api::rules::RoadRulebook* rulebook,
                                                     const api::rules::PhaseRingBook* phase_ring_book,
                                                     const api::rules::PhaseProvider* phase_provider);

  ~PhaseBasedRightOfWayDiscreteValueRuleStateProvider() final = default;

  const api::rules::PhaseRingBook& phase_ring_book() const { return *phase_ring_book_; }

  const api::rules::PhaseProvider& phase_provider() const { return *phase_provider_; }

 private:
  // Gets the state for Right-Of-Way rules as the class docstring explain. When
  // nothing can be told about the rule based on the phase,
  // ManualDiscreteValueRuleStateProvider::DoGetState() is called.
  std::optional<api::rules::DiscreteValueRuleStateProvider::StateResult> DoGetState(
      const api::rules::Rule::Id& id) const final;

  // Gets the state for a `rule_type` rule at `road_position` under certain `tolerance`, as the class docstring explain.
  // When nothing can be told about the rule based on the phase,
  // ManualDiscreteValueRuleStateProvider::DoGetState() is called.
  std::optional<api::rules::DiscreteValueRuleStateProvider::StateResult> DoGetState(
      const api::RoadPosition& road_position, const api::rules::Rule::TypeId& rule_type, double tolerance) const final;

  const api::rules::PhaseRingBook* phase_ring_book_{};
  const api::rules::PhaseProvider* phase_provider_{};
};

}  // namespace maliput
