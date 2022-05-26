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

#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/// Provides the state of api::rules::RightOfWayRule instances based on
/// the current api::rules::Phase. The states of the rules that govern an
/// intersection are organized into phases. Each phase typically assigns
/// different states to each rule to ensure intersection safety and fairness.
/// For example, given an intersection between streets A and B governed by
/// Rule_A and Rule_B, respectively, two phases are necessary:
///
/// Phase  | Rule_A State | Rule_B State
/// ------ | ------------ | ------------
///   1    | GO           | STOP
///   2    | STOP         | GO
///
/// The rules above will ensure vehicles traveling on Street A do not collide
/// with vehicles traveling on Street B and vice versa.
class MALIPUT_DEPRECATED("api::rules::RightOfWayRuleStateProvider class will be deprecated.",
                         "PhasedDiscreteRuleStateProvider") PhaseBasedRightOfWayRuleStateProvider final
    : public api::rules::RightOfWayRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseBasedRightOfWayRuleStateProvider)

  /// Constructs a PhaseBasedRuleStateProvider.
  ///
  /// All pointer parameters are aliased; they must not be nullptr and their
  /// lifespans must exceed that of this instance.
  PhaseBasedRightOfWayRuleStateProvider(const api::rules::PhaseRingBook* phase_ring_book,
                                        const api::rules::PhaseProvider* phase_provider);

  ~PhaseBasedRightOfWayRuleStateProvider() final = default;

  const api::rules::PhaseRingBook& phase_ring_book() const { return *phase_ring_book_; }

  const api::rules::PhaseProvider& phase_provider() const { return *phase_provider_; }

 private:
  std::optional<api::rules::RightOfWayRuleStateProvider::RightOfWayResult> DoGetState(
      const api::rules::RightOfWayRule::Id& id) const final;

  const api::rules::PhaseRingBook* phase_ring_book_{};
  const api::rules::PhaseProvider* phase_provider_{};
};
#pragma GCC diagnostic pop

}  // namespace maliput
