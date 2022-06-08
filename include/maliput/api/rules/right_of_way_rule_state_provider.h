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

#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/state_provider_result.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the provider of the RightOfWayRule.
class MALIPUT_DEPRECATED("RigthOfWayRule class will be deprecated.") RightOfWayRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayRuleStateProvider)

  virtual ~RightOfWayRuleStateProvider() = default;

  /// Result returned by GetState(const RightOfWayRule::Id).
  using RightOfWayResult = StateProviderResult<RightOfWayRule::State::Id>;

  /// Gets the state of the RightOfWayRule identified by `id`.
  ///
  /// Returns a RightOfWayResult struct bearing the State::Id of the rule's
  /// current state.  If a transition to a new state is anticipated,
  /// RightOfWayResult::next will be populated and bear the State::Id of the
  /// next state.  If the time until the transition is known, then
  /// RightOfWayResult::next.duration_until will be populated with that
  /// duration.
  ///
  /// Returns std::nullopt if `id` is unrecognized, which would be the case
  /// if no such rule exists or if the rule has only static semantics.
  std::optional<RightOfWayResult> GetState(const RightOfWayRule::Id& id) const { return DoGetState(id); }

 protected:
  RightOfWayRuleStateProvider() = default;

 private:
  virtual std::optional<RightOfWayResult> DoGetState(const RightOfWayRule::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
