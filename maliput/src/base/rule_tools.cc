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
#include "maliput/base/rule_tools.h"

#include <optional>

#include "maliput/api/rules/state_provider_result.h"
#include "maliput/base/rule_registry.h"

namespace maliput {

namespace {

// Returns the current state for a given Right-Of-Way Rule.
maliput::api::rules::DiscreteValueRule::DiscreteValue GetCurrentStateValue(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  MALIPUT_THROW_UNLESS(state_provider != nullptr);
  MALIPUT_THROW_UNLESS(discrete_value_rule.type_id() == RightOfWayRuleTypeId());
  const std::optional<maliput::api::rules::DiscreteValueRuleStateProvider::StateResult> state_result{
      state_provider->GetState(discrete_value_rule.id())};
  MALIPUT_THROW_UNLESS(state_result != std::nullopt);
  return state_result->state;
}

}  // namespace

std::vector<maliput::api::rules::Rule::Id> GetYieldGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
  MALIPUT_THROW_UNLESS(discrete_value.related_rules.find(RelatedRulesKeys::kYieldGroup) !=
                       discrete_value.related_rules.end());
  return discrete_value.related_rules.at(RelatedRulesKeys::kYieldGroup);
}

std::vector<maliput::api::rules::Rule::Id> GetCurrentYieldGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  return GetYieldGroup(GetCurrentStateValue(discrete_value_rule, state_provider));
}

std::vector<maliput::api::UniqueId> GetBulbGroup(
    const maliput::api::rules::DiscreteValueRule::DiscreteValue& discrete_value) {
  MALIPUT_THROW_UNLESS(discrete_value.related_unique_ids.find(RelatedUniqueIdsKeys::kBulbGroup) !=
                       discrete_value.related_unique_ids.end());
  return discrete_value.related_unique_ids.at(RelatedUniqueIdsKeys::kBulbGroup);
}

std::vector<maliput::api::UniqueId> GetCurrentBulbGroup(
    const maliput::api::rules::DiscreteValueRule& discrete_value_rule,
    const maliput::api::rules::DiscreteValueRuleStateProvider* state_provider) {
  return GetBulbGroup(GetCurrentStateValue(discrete_value_rule, state_provider));
}

}  // namespace maliput
