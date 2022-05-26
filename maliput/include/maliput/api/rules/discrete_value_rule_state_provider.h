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

#include "maliput/api/lane_data.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/state_provider_result.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for the state provider of DiscreteValueRules.
class DiscreteValueRuleStateProvider {
 public:
  /// The state of a DiscreteValueRule, returned by
  /// DiscreteValueRuleStateProvider::GetState(const Rule::Id&).
  using StateResult = StateProviderResult<DiscreteValueRule::DiscreteValue>;

  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteValueRuleStateProvider)

  virtual ~DiscreteValueRuleStateProvider() = default;

  /// Gets the state of the DiscreteValueRule identified by `id`.
  ///
  /// Returns a StateResult struct bearing the state of the rule with the
  /// specified `id`. If `id` is unrecognized, std::nullopt is returned.
  std::optional<StateResult> GetState(const Rule::Id& id) const { return DoGetState(id); }

  /// Gets the state of the DiscreteValueRule that matches provided `road_position` and `rule_type` values according to
  /// certain `tolerance`.
  ///
  /// Returns a StateResult struct bearing the state of the rule.
  /// If no rule is obtained out of the `road_position` and `rule_type` combination, std::nullopt is returned.
  std::optional<StateResult> GetState(const RoadPosition& road_position, const Rule::TypeId& rule_type,
                                      double tolerance) const {
    return DoGetState(road_position, rule_type, tolerance);
  }

 protected:
  DiscreteValueRuleStateProvider() = default;

 private:
  virtual std::optional<StateResult> DoGetState(const Rule::Id& id) const = 0;
  virtual std::optional<StateResult> DoGetState(const RoadPosition& road_position, const Rule::TypeId& rule_type,
                                                double tolerance) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
