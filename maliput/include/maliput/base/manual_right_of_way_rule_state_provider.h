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

#include <unordered_map>

#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// A trivial implementation of an api::rules::RightOfWayRuleStateProvider.
class MALIPUT_DEPRECATED("api::rules::RightOfWayRuleStateProvider class will be deprecated.",
                         "ManualDiscreteValueRuleStateProvider") ManualRightOfWayRuleStateProvider final
    : public api::rules::RightOfWayRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ManualRightOfWayRuleStateProvider)

  /// Default constructor.
  ManualRightOfWayRuleStateProvider() {}

  ~ManualRightOfWayRuleStateProvider() final = default;

  /// Adds a dynamic state to this provider.
  ///
  /// @throws std::logic_error if a RightOfWayRule with an ID of @p id
  /// already exists in this provider.
  /// @throws maliput::common::assertion_error if the dynamic state failed to be
  /// added.
  void AddState(const api::rules::RightOfWayRule::Id& id, const api::rules::RightOfWayRule::State::Id& initial_state);

  /// Sets the dynamic state of a RightOfWayRule within this provider.
  ///
  /// @throws std::out_of_range if no dynamic state with @p id exists in this
  /// provider.
  void SetState(const api::rules::RightOfWayRule::Id& id, const api::rules::RightOfWayRule::State::Id& state);

 private:
  std::optional<api::rules::RightOfWayRuleStateProvider::RightOfWayResult> DoGetState(
      const api::rules::RightOfWayRule::Id& id) const final;

  std::unordered_map<api::rules::RightOfWayRule::Id, api::rules::RightOfWayRule::State::Id> states_;
};
#pragma GCC diagnostic pop

}  // namespace maliput
