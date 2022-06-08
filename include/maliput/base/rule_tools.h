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

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/discrete_value_rule_state_provider.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/rule_registry.h"

namespace maliput {

/// Returns a vector of Rule::Ids that represents the yield group of a Right-Of-Way rule state.
/// @throws common::assertion_error When `discrete_value.related_rules` does not have RelatedRulesKeys::kYieldGroup
/// key.
std::vector<maliput::api::rules::Rule::Id> GetYieldGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value);

/// Returns the current yield group of `discrete_value_rule`.
/// @param discrete_value_rule A Right-Of-Way Rule Type rule.
/// @param state_provider A state provider to retrieve current `discrete_value_rule` state. It must not be nullptr.
/// @throws common::assertion_error When `discrete_value_rule.type_id()` is not RightOfWayRuleTypeId().
/// @throws common::assertion_error When `state_provider` is nullptr.
/// @throws common::assertion_error When `state_provider` does not hold any state for `discrete_value_rule`.
std::vector<maliput::api::rules::Rule::Id> GetCurrentYieldGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider);

/// Returns a vector of maliput::api::UniqueIds that represents the bulb group of a Right-Of-Way rule state.
/// @throws common::assertion_error When `discrete_value.related_unique_ids` does not have
/// RelatedUniqueIdsKeys::kBulbGroup key.
std::vector<maliput::api::UniqueId> GetBulbGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value);

/// Returns the current bulb group of `discrete_value_rule`.
/// @param discrete_value_rule A Right-Of-Way Rule Type rule.
/// @param state_provider A state provider to retrieve current `discrete_value_rule` state. It must not be nullptr.
/// @throws common::assertion_error When `discrete_value_rule.type_id()` is not RightOfWayRuleTypeId().
/// @throws common::assertion_error When `state_provider` is nullptr.
/// @throws common::assertion_error When `state_provider` does not hold any state for `discrete_value_rule`.
std::vector<maliput::api::UniqueId> GetCurrentBulbGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider);

/// Functor to filter by api::rules::Rule::TypeId.
struct RuleTypeFilter {
  RuleTypeFilter(const api::rules::Rule::TypeId& rule_type_in) : rule_type(rule_type_in) {}

  bool operator()(const api::rules::Rule& rule) { return rule.type_id() == rule_type; }

  api::rules::Rule::TypeId rule_type;
};

}  // namespace maliput
