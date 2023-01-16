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
#include "maliput/base/phased_discrete_rule_state_provider.h"

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/base/rule_filter.h"
#include "maliput/common/logger.h"
#include "maliput/common/maliput_abort.h"

namespace maliput {

using api::rules::DiscreteValueRule;
using api::rules::DiscreteValueRuleStateProvider;
using api::rules::Phase;
using api::rules::PhaseProvider;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::Rule;

namespace {

// Sets to `state_provider` the first value in each
// DiscreteValueRule::states() in `discrete_value_rules` as the default
// state.
// @throws maliput::common::assertion_error When `state_provider` is
//         nullptr.
void PopulateDiscreteValueRuleStates(const std::map<Rule::Id, DiscreteValueRule>& discrete_value_rules,
                                     maliput::PhasedDiscreteRuleStateProvider* state_provider) {
  MALIPUT_THROW_UNLESS(state_provider != nullptr);
  for (const auto& rule_id_rule : discrete_value_rules) {
    state_provider->SetState(rule_id_rule.first, rule_id_rule.second.states().front(), std::nullopt, std::nullopt);
  }
}

}  // namespace

std::unique_ptr<PhasedDiscreteRuleStateProvider>
PhasedDiscreteRuleStateProvider::GetDefaultPhasedDiscreteRuleStateProvider(
    const maliput::api::rules::RoadRulebook* rulebook, const maliput::api::rules::PhaseRingBook* phase_ring_book,
    const maliput::api::rules::PhaseProvider* phase_provider) {
  auto state_provider =
      std::make_unique<maliput::PhasedDiscreteRuleStateProvider>(rulebook, phase_ring_book, phase_provider);
  const maliput::api::rules::RoadRulebook::QueryResults all_rules = rulebook->Rules();
  PopulateDiscreteValueRuleStates(all_rules.discrete_value_rules, state_provider.get());
  return state_provider;
}

PhasedDiscreteRuleStateProvider::PhasedDiscreteRuleStateProvider(const api::rules::RoadRulebook* rulebook,
                                                                 const PhaseRingBook* phase_ring_book,
                                                                 const PhaseProvider* phase_provider)
    : ManualDiscreteValueRuleStateProvider(rulebook),
      phase_ring_book_(phase_ring_book),
      phase_provider_(phase_provider) {
  MALIPUT_THROW_UNLESS(phase_ring_book_ != nullptr);
  MALIPUT_THROW_UNLESS(phase_provider_ != nullptr);
}

std::optional<DiscreteValueRuleStateProvider::StateResult> PhasedDiscreteRuleStateProvider::DoGetState(
    const Rule::Id& rule_id) const {
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

std::optional<api::rules::DiscreteValueRuleStateProvider::StateResult> PhasedDiscreteRuleStateProvider::DoGetState(
    const api::RoadPosition& road_position, const api::rules::Rule::TypeId& rule_type, double tolerance) const {
  const auto filtered_discrete_value_rules = GetFilteredDiscreteValueRules(road_position, rule_type, tolerance);
  if (filtered_discrete_value_rules.empty()) {
    // Returns empty state result if no rule is found.
    return {};
  }
  if (filtered_discrete_value_rules.size() > 1) {
    maliput::log()->warn(
        "For rule_type: {} and road_position: [LaneId: {}, LanePos: {}] there are more than one possible rules: ",
        rule_type.string(), road_position.lane->id(), road_position.pos.srh().to_str());
    for (const auto& rule : filtered_discrete_value_rules) {
      maliput::log()->warn("\tRule id: {} matches with rule_type: {} and road_position: [LaneId: {}, LanePos: {}]",
                           rule.first.string(), rule_type.string(), road_position.lane->id(),
                           road_position.pos.srh().to_str());
    }
  }
  // Once we have the rule id we can leverage DoGetState(const Rule::Id& rule_id) method.
  return DoGetState(filtered_discrete_value_rules.begin()->first);
}

}  // namespace maliput
