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
#include "maliput/api/rules/phase_ring.h"

#include <utility>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

/// Tests for completeness. The set of RightOfWayRule::Id and Bulb::Id
/// referenced by every phase must be the same. In other words, every Rule::Id
/// or Bulb::Id referenced in any one phase must be referenced in all phases.
/// This is because every phase must specify the complete state of all the rules
/// and bulb states mentioned by the ring.
void VerifyAllPhasesHaveSameCoverage(const PhaseRing::Id& id, const std::vector<Phase>& phases) {
  MALIPUT_VALIDATE(phases.size() >= 1, "PhaseRing(" + id.string() + ") must have at least one phase.");
  const auto& r = phases.at(0);  // The reference phase.
  for (const auto& phase : phases) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    MALIPUT_VALIDATE(phase.rule_states().size() == r.rule_states().size(),
                     "PhaseRing(" + id.string() + "). Phase(" + phase.id().string() + ") and Phase(" + r.id().string() +
                         ") must have the same number of rule_states.");
    for (const auto& s : phase.rule_states()) {
      MALIPUT_VALIDATE(r.rule_states().count(s.first) == 1, "PhaseRing(" + id.string() + "). Phase(" +
                                                                phase.id().string() + ") has duplicated rule states.");
    }
#pragma GCC diagnostic pop
    MALIPUT_VALIDATE(phase.discrete_value_rule_states().size() == r.discrete_value_rule_states().size(),
                     "PhaseRing(" + id.string() + "). Phase(" + phase.id().string() + ") and Phase(" + r.id().string() +
                         ") must have the same number of discrete_value_rule_states.");
    for (const auto& s : phase.discrete_value_rule_states()) {
      MALIPUT_VALIDATE(r.discrete_value_rule_states().count(s.first) == 1,
                       "PhaseRing(" + id.string() + "). Phase(" + phase.id().string() +
                           ") has duplicated discrete value rule states");
    }
    // Require both set of bulb states to be defined or undefined together.
    if (r.bulb_states() != std::nullopt) {
      MALIPUT_VALIDATE(phase.bulb_states()->size() == r.bulb_states()->size(),
                       "Phase(" + id.string() + "). Phase(" + phase.id().string() + ") and Phase(" + r.id().string() +
                           ") must have the same number of bulb_states.");
      for (const auto& s : *phase.bulb_states()) {
        MALIPUT_VALIDATE(r.bulb_states()->count(s.first) == 1,
                         "PhaseRing(" + id.string() + ") has duplicated bulb states");
      }
    }
  }
}

/// Tests that `next_phases` defines the possible next phases of every
/// phase in `phases` and nothing more.
void VerifyNextPhases(const PhaseRing::Id& id, const std::vector<Phase>& phases,
                      const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>>& next_phases) {
  MALIPUT_VALIDATE(phases.size() == next_phases.size(),
                   "PhaseRing(" + id.string() + ")'s phases and next_phases sizes must be equal.");
  for (const auto& phase : phases) {
    MALIPUT_VALIDATE(
        next_phases.find(phase.id()) != next_phases.end(),
        "In PhaseRing(" + id.string() + "), the Phase(" + phase.id().string() + ") is not in next_phases.");
  }
}

}  // namespace

PhaseRing::PhaseRing(const Id& id, const std::vector<Phase>& phases,
                     const std::optional<const std::unordered_map<Phase::Id, std::vector<NextPhase>>>& next_phases)
    : id_(id) {
  MALIPUT_VALIDATE(phases.size() >= 1, "PhaseRing(" + id_.string() + ") must have at least one phase.");
  for (const Phase& phase : phases) {
    // Construct index of phases by ID, ensuring uniqueness of ID's.
    auto result = phases_.emplace(phase.id(), phase);
    MALIPUT_THROW_UNLESS(result.second);
  }
  if (next_phases != std::nullopt) {
    next_phases_ = *next_phases;
    VerifyNextPhases(id_, phases, next_phases_);
  } else {
    for (const auto& phase : phases) {
      next_phases_.emplace(std::make_pair(phase.id(), std::vector<NextPhase>()));
    }
  }
  VerifyAllPhasesHaveSameCoverage(id_, phases);
}

std::optional<Phase> PhaseRing::GetPhase(const Phase::Id& id) const {
  return phases_.find(id) != phases_.end() ? std::optional<Phase>(phases_.at(id)) : std::nullopt;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
