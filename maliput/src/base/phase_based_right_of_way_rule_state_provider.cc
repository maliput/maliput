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
#include "maliput/base/phase_based_right_of_way_rule_state_provider.h"

#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/common/maliput_abort.h"

namespace maliput {

using api::rules::Phase;
using api::rules::PhaseProvider;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RightOfWayRule;
using api::rules::RightOfWayRuleStateProvider;

PhaseBasedRightOfWayRuleStateProvider::PhaseBasedRightOfWayRuleStateProvider(const PhaseRingBook* phase_ring_book,
                                                                             const PhaseProvider* phase_provider)
    : phase_ring_book_(phase_ring_book), phase_provider_(phase_provider) {
  MALIPUT_DEMAND(phase_ring_book_ != nullptr && phase_provider != nullptr);
}

std::optional<RightOfWayRuleStateProvider::RightOfWayResult> PhaseBasedRightOfWayRuleStateProvider::DoGetState(
    const RightOfWayRule::Id& rule_id) const {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::optional<PhaseRing> ring = phase_ring_book_->FindPhaseRing(rule_id);
#pragma GCC diagnostic pop
  if (ring.has_value()) {
    const std::optional<PhaseProvider::Result> phase_result = phase_provider_->GetPhase(ring->id());
    if (phase_result.has_value()) {
      const Phase::Id phase_id = phase_result->state;
      const Phase& phase = ring->phases().at(phase_id);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
      const RightOfWayRule::State::Id state_id = phase.rule_states().at(rule_id);
      std::optional<RightOfWayResult::Next> next = std::nullopt;
#pragma GCC diagnostic pop
      if (phase_result->next.has_value()) {
        const Phase::Id next_phase_id = phase_result->next->state;
        const Phase& next_phase = ring->phases().at(next_phase_id);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        const RightOfWayRule::State::Id next_state_id = next_phase.rule_states().at(rule_id);
        next = RightOfWayResult::Next{next_state_id, phase_result->next->duration_until};
      }
      return RightOfWayResult{state_id, next};
    }
  }
#pragma GCC diagnostic pop
  return std::nullopt;
}

}  // namespace maliput
