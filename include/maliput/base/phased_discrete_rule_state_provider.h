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
class PhasedDiscreteRuleStateProvider final : public ManualDiscreteValueRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(PhasedDiscreteRuleStateProvider)

  /// Constructs a PhasedDiscreteRuleStateProvider with the default states populated to be used when rules are not parte
  /// of a phase.
  ///
  /// @param rulebook The RoadRulebook to use.
  /// @param phase_ring_book The PhaseRingBook to use.
  /// @param phase_provider The PhaseProvider to use.
  /// @throws maliput::common::assertion_error When phase_ring_book is nullptr.
  static std::unique_ptr<PhasedDiscreteRuleStateProvider> GetDefaultPhasedDiscreteRuleStateProvider(
      const maliput::api::rules::RoadRulebook* rulebook, const maliput::api::rules::PhaseRingBook* phase_ring_book,
      const maliput::api::rules::PhaseProvider* phase_provider);

  /// Constructs a PhasedDiscreteRuleStateProvider.
  ///
  /// All pointer parameters are aliased; they must not be nullptr and their
  /// lifespans must exceed that of this instance.
  ///
  /// @throws common::assertion_error When `rulebook` is nullptr.
  /// @throws common::assertion_error When `phase_ring_book` is nullptr.
  /// @throws common::assertion_error When `phase_provider` is nullptr.
  PhasedDiscreteRuleStateProvider(const api::rules::RoadRulebook* rulebook,
                                  const api::rules::PhaseRingBook* phase_ring_book,
                                  const api::rules::PhaseProvider* phase_provider);

  ~PhasedDiscreteRuleStateProvider() final = default;

  const api::rules::PhaseRingBook& phase_ring_book() const { return *phase_ring_book_; }

  const api::rules::PhaseProvider& phase_provider() const { return *phase_provider_; }

 private:
  // Gets the state for Right-Of-Way rules as the class docstring explains. When
  // nothing can be told about the rule based on the phase,
  // ManualDiscreteValueRuleStateProvider::DoGetState() is called.
  std::optional<api::rules::DiscreteValueRuleStateProvider::StateResult> DoGetState(
      const api::rules::Rule::Id& id) const final;

  // Gets the state for a `rule_type` rule at `road_position` under certain `tolerance`, as the class docstring
  // explains.
  // Some considerations:
  //  - When nothing can be told about the rule based on the phase, ManualDiscreteValueRuleStateProvider::DoGetState()
  //  is called.
  //  - When per `road_position` and `rule_type` more than one rule is applicable, the state of the first applicable
  //  rule is returned. In this case an appropriate warning is logged as it is an unusual case.
  std::optional<api::rules::DiscreteValueRuleStateProvider::StateResult> DoGetState(
      const api::RoadPosition& road_position, const api::rules::Rule::TypeId& rule_type, double tolerance) const final;

  const api::rules::PhaseRingBook* phase_ring_book_{};
  const api::rules::PhaseProvider* phase_provider_{};
};

}  // namespace maliput
