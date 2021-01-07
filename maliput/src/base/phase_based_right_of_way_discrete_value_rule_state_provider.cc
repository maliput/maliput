#include "maliput/base/phase_based_right_of_way_discrete_value_rule_state_provider.h"

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/common/maliput_abort.h"

namespace maliput {

using api::rules::DiscreteValueRule;
using api::rules::DiscreteValueRuleStateProvider;
using api::rules::Phase;
using api::rules::PhaseProvider;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::Rule;

PhaseBasedRightOfWayDiscreteValueRuleStateProvider::PhaseBasedRightOfWayDiscreteValueRuleStateProvider(
    const api::rules::RoadRulebook* rulebook, const PhaseRingBook* phase_ring_book, const PhaseProvider* phase_provider)
    : ManualDiscreteValueRuleStateProvider(rulebook),
      phase_ring_book_(phase_ring_book),
      phase_provider_(phase_provider) {
  MALIPUT_THROW_UNLESS(phase_ring_book_ != nullptr);
  MALIPUT_THROW_UNLESS(phase_provider_ != nullptr);
}

std::optional<DiscreteValueRuleStateProvider::StateResult>
PhaseBasedRightOfWayDiscreteValueRuleStateProvider::DoGetState(const Rule::Id& rule_id) const {
  std::optional<PhaseRing> ring = phase_ring_book_->FindPhaseRing(rule_id);
  if (ring.has_value()) {
    const std::optional<PhaseProvider::Result> phase_result = phase_provider_->GetPhase(ring->id());
    if (phase_result.has_value()) {
      const Phase::Id phase_id = phase_result->state;
      const Phase& phase = ring->phases().at(phase_id);
      const DiscreteValueRule::DiscreteValue value = phase.discrete_value_rule_states().at(rule_id);
      std::optional<DiscreteValueRuleStateProvider::StateResult::Next> next = std::nullopt;
      if (phase_result->next.has_value()) {
        const Phase::Id next_phase_id = phase_result->next->state;
        const Phase& next_phase = ring->phases().at(next_phase_id);
        const DiscreteValueRule::DiscreteValue next_value = next_phase.discrete_value_rule_states().at(rule_id);
        next = DiscreteValueRuleStateProvider::StateResult::Next{next_value, phase_result->next->duration_until};
      }
      return DiscreteValueRuleStateProvider::StateResult{value, next};
    }
  }
  return ManualDiscreteValueRuleStateProvider::DoGetState(rule_id);
}

}  // namespace maliput
