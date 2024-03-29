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
#include <unordered_map>

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/range_value_rule_state_provider.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {

/// An implementation of a manual api::rules::RangeValueRuleStateProvider.
/// This enables clients to directly set the states of
/// api::rules::RangeValueRule instances.
class ManualRangeValueRuleStateProvider : public api::rules::RangeValueRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ManualRangeValueRuleStateProvider)

  /// Constructs a ManualRangeValueRuleStateProvider with the default states populated.
  ///
  /// @param rulebook The RoadRulebook to use.
  static std::unique_ptr<ManualRangeValueRuleStateProvider> GetDefaultManualRangeValueRuleStateProvider(
      const maliput::api::rules::RoadRulebook* rulebook);

  /// Constructs a ManualRangeValueRuleStateProvider.
  ///
  /// @param rulebook A rulebook pointer to validate Rule::Id and their states.
  ///        It must not be nullptr.
  ///
  /// @throws common::assertion_error When `rulebook` is nullptr.
  explicit ManualRangeValueRuleStateProvider(const api::rules::RoadRulebook* rulebook)
      : api::rules::RangeValueRuleStateProvider(), rulebook_(rulebook) {
    MALIPUT_THROW_UNLESS(rulebook_ != nullptr);
  }

  ~ManualRangeValueRuleStateProvider() override = default;

  /// Sets the state, and optionally the next state and duration until the next
  /// state, of a RangeValueRule.
  ///
  /// If @p id is new, a new entry is added to this provider.
  ///
  /// @p next_state and @p duration_until are not managed by this
  /// provider, i.e. it is the client's responsibility to govern transitions from
  /// @p state to @p next_state after @p duration_until (if provided) time.
  ///
  /// @param id RangeValueRule's ID. It must be recognized by the `rulebook`
  ///        specified at time of construction.
  /// @param state The state of the rule. It must be one in
  ///        RangeValueRule::states().
  /// @param next_state The optional next state of the rule. It must be one in
  ///        RangeValueRule::states().
  /// @param duration_until If known, the estimated time until the transition to
  ///        the next state, relative to when this method is called. When
  ///        provided, it must be positive and @p next_state must not be nullopt.
  ///
  /// @throws std::out_of_range When @p id is unrecognized by the `rulebook`
  ///         specified at time of construction.
  /// @throws common::assertion_error When @p state does not match any state
  ///         in RangeValueRule::states().
  /// @throws common::assertion_error When @p next_state does not match any state
  ///         in RangeValueRule::states().
  /// @throws common::assertion_error When @p duration_until is not positive or
  ///         it is provided when @p next_state is nullopt.
  void SetState(const api::rules::Rule::Id& id, const api::rules::RangeValueRule::Range& state,
                const std::optional<api::rules::RangeValueRule::Range>& next_state,
                const std::optional<double>& duration_until);

 private:
  // This function has been marked as virtual because other providers might
  // benefit from injecting their own getter and then forwarding calls to this
  // function.
  virtual std::optional<api::rules::RangeValueRuleStateProvider::StateResult> DoGetState(
      const api::rules::Rule::Id& id) const override;

  // This function has been marked as virtual because other providers might
  // benefit from injecting their own getter and then forwarding calls to this
  // function.
  virtual std::optional<api::rules::RangeValueRuleStateProvider::StateResult> DoGetState(
      const api::RoadPosition& road_position, const api::rules::Rule::TypeId& rule_type,
      double tolerance) const override;

  // @throws common::assertion_error When @p state is unrecognized in
  //         @p range_value_rule's ranges.
  void ValidateRuleState(const api::rules::RangeValueRule& range_value_rule,
                         const api::rules::RangeValueRule::Range& state) const;

  std::unordered_map<api::rules::Rule::Id, api::rules::RangeValueRuleStateProvider::StateResult> states_;
  const api::rules::RoadRulebook* rulebook_{nullptr};
};

}  // namespace maliput
