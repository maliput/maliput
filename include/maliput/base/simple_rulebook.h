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

#include <memory>
#include <vector>

#include "maliput/api/regions.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {

/// SimpleRulebook is a simple concrete implementation of the
/// api::rules::RoadRulebook abstract interface.
class SimpleRulebook : public api::rules::RoadRulebook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleRulebook);

  /// Constructs an empty SimpleRulebook (i.e., containing no rules).
  SimpleRulebook();

  ~SimpleRulebook() override;

  /// Removes all rules.
  void RemoveAll();

  /// Adds a new RightOfWayRule.
  ///
  /// @throws maliput::common::assertion_error if a rule with the same ID
  /// already exists in the SimpleRulebook.
  void AddRule(const api::rules::RightOfWayRule& rule);

  /// Removes the RightOfWayRule labeled by `id`.
  ///
  /// @throws maliput::common::assertion_error if no such rule exists.
  void RemoveRule(const api::rules::RightOfWayRule::Id& id);

  /// Adds a new SpeedLimitRule.
  ///
  /// @throws maliput::common::assertion_error if a rule with the same ID
  /// already exists in the SimpleRulebook.
  void AddRule(const api::rules::SpeedLimitRule& rule);

  /// Removes the SpeedLimitRule labeled by `id`.
  ///
  /// @throws maliput::common::assertion_error if no such rule exists.
  void RemoveRule(const api::rules::SpeedLimitRule::Id& id);

  /// Adds a new DirectionUsageRule.
  ///
  /// @throws maliput::common::assertion_error if a rule with the same ID
  /// already exists in the SimpleRulebook.
  void AddRule(const api::rules::DirectionUsageRule& rule);

  /// Removes the DirectionUsageRule labeled by `id`.
  ///
  /// @throws maliput::common::assertion_error if no such rule exists.
  void RemoveRule(const api::rules::DirectionUsageRule::Id& id);

 private:
  api::rules::RoadRulebook::QueryResults DoFindRules(const std::vector<api::LaneSRange>& ranges,
                                                     double tolerance) const override;
  api::rules::RightOfWayRule DoGetRule(const api::rules::RightOfWayRule::Id& id) const override;
  api::rules::SpeedLimitRule DoGetRule(const api::rules::SpeedLimitRule::Id& id) const override;
  api::rules::DirectionUsageRule DoGetRule(const api::rules::DirectionUsageRule::Id& id) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
