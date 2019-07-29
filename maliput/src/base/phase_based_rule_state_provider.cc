#include "maliput/base/phase_based_rule_state_provider.h"

#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/common/maliput_abort.h"

namespace maliput {

using api::rules::Phase;
using api::rules::PhaseProvider;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RightOfWayRule;
using api::rules::RuleStateProvider;

PhaseBasedRuleStateProvider::PhaseBasedRuleStateProvider(
    const PhaseRingBook* phase_ring_book, const PhaseProvider* phase_provider)
    : phase_ring_book_(phase_ring_book), phase_provider_(phase_provider) {
  MALIPUT_DEMAND(phase_ring_book_ != nullptr && phase_provider != nullptr);
}

drake::optional<RuleStateProvider::RightOfWayResult>
PhaseBasedRuleStateProvider::DoGetState(const RightOfWayRule::Id& rule_id)
    const {
  drake::optional<PhaseRing> ring = phase_ring_book_->FindPhaseRing(rule_id);
  if (ring.has_value()) {
    const drake::optional<PhaseProvider::Result> phase_result =
        phase_provider_->GetPhase(ring->id());
    if (phase_result.has_value()) {
      const Phase::Id phase_id = phase_result->id;
      const Phase& phase = ring->phases().at(phase_id);
      const RightOfWayRule::State::Id state_id =
          phase.rule_states().at(rule_id);
      drake::optional<RightOfWayResult::Next> next = drake::nullopt;
      if (phase_result->next.has_value()) {
        const Phase::Id next_phase_id = phase_result->next->id;
        const Phase& next_phase = ring->phases().at(next_phase_id);
        const RightOfWayRule::State::Id next_state_id =
            next_phase.rule_states().at(rule_id);
        next = RightOfWayResult::Next{next_state_id,
                                      phase_result->next->duration_until};
      }
      return RightOfWayResult{state_id, next};
    }
  }
  return drake::nullopt;
}

}  // namespace maliput
