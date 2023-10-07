// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota.
// All rights reserved.
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
#include <string>
#include <vector>

#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/common/compare.h"

namespace maliput {
namespace api {
namespace rules {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Evaluates the equality of two RuleStates.
/// @param a The first RuleStates to compare.
/// @param b The second RuleStates to compare.
/// @returns A ComparisonResult indicating whether the two RuleStates are equal.
common::ComparisonResult<RuleStates> IsEqual(const RuleStates& a, const RuleStates& b);
#pragma GCC diagnostic pop

/// Evaluates the equality of two Phase.
/// @param a The first Phase to compare.
/// @param b The second Phase to compare.
/// @returns A ComparisonResult indicating whether the two Phase are equal.
common::ComparisonResult<Phase> IsEqual(const Phase& a, const Phase& b);

/// Evaluates the equality of two PhaseRing::NextPhase.
/// @param a The first PhaseRing::NextPhase to compare.
/// @param b The second PhaseRing::NextPhase to compare.
/// @returns A ComparisonResult indicating whether the two PhaseRing::NextPhase are equal.
common::ComparisonResult<PhaseRing::NextPhase> IsEqual(const PhaseRing::NextPhase& a, const PhaseRing::NextPhase& b);

/// Evaluates the equality of two std::vector<PhaseRing::NextPhase>.
/// @param a The first std::vector<PhaseRing::NextPhase> to compare.
/// @param b The second std::vector<PhaseRing::NextPhase> to compare.
/// @returns A ComparisonResult indicating whether the two std::vector<PhaseRing::NextPhase> are equal.
common::ComparisonResult<std::vector<PhaseRing::NextPhase>> IsEqual(const std::vector<PhaseRing::NextPhase>& a,
                                                                    const std::vector<PhaseRing::NextPhase>& b);

/// Evaluates the equality of two RangeValueRule::Range.
/// @param a The first RangeValueRule::Range to compare.
/// @param b The second RangeValueRule::Range to compare.
/// @returns A ComparisonResult indicating whether the two RangeValueRule::Range are equal.
common::ComparisonResult<RangeValueRule::Range> IsEqual(const RangeValueRule::Range& a, const RangeValueRule::Range& b);

/// Evaluates the equality of two RangeValueRule.
/// @param a The first RangeValueRule to compare.
/// @param b The second RangeValueRule to compare.
/// @returns A ComparisonResult indicating whether the two RangeValueRule are equal.
common::ComparisonResult<RangeValueRule> IsEqual(const RangeValueRule& a, const RangeValueRule& b);

/// Evaluates the equality of two std::vector<RangeValueRule::Range>.
/// @param a The first std::vector<RangeValueRule::Range> to compare.
/// @param b The second std::vector<RangeValueRule::Range> to compare.
/// @returns A ComparisonResult indicating whether the two std::vector<RangeValueRule::Range> are equal.
common::ComparisonResult<std::vector<RangeValueRule::Range>> IsEqual(const std::vector<RangeValueRule::Range>& a,
                                                                     const std::vector<RangeValueRule::Range>& b);

/// Evaluates the equality of two DiscreteValueRule::DiscreteValue.
/// @param a The first DiscreteValueRule::DiscreteValue to compare.
/// @param b The second DiscreteValueRule::DiscreteValue to compare.
/// @returns A ComparisonResult indicating whether the two DiscreteValueRule::DiscreteValue are equal.
common::ComparisonResult<DiscreteValueRule::DiscreteValue> IsEqual(const DiscreteValueRule::DiscreteValue& a,
                                                                   const DiscreteValueRule::DiscreteValue& b);

/// Evaluates the equality of two DiscreteValueRule.
/// @param a The first DiscreteValueRule to compare.
/// @param b The second DiscreteValueRule to compare.
/// @returns A ComparisonResult indicating whether the two DiscreteValueRule are equal.
common::ComparisonResult<DiscreteValueRule> IsEqual(const DiscreteValueRule& a, const DiscreteValueRule& b);

/// Evaluates the equality of two std::vector<DiscreteValueRule::DiscreteValue>.
/// @param a The first std::vector<DiscreteValueRule::DiscreteValue> to compare.
/// @param b The second std::vector<DiscreteValueRule::DiscreteValue> to compare.
/// @returns A ComparisonResult indicating whether the two std::vector<DiscreteValueRule::DiscreteValue> are equal.
common::ComparisonResult<std::vector<DiscreteValueRule::DiscreteValue>> IsEqual(
    const std::vector<DiscreteValueRule::DiscreteValue>& a, const std::vector<DiscreteValueRule::DiscreteValue>& b);

/// Evaluates the equality of two std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>.
/// @param a The first std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue> to compare.
/// @param b The second std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue> to compare.
/// @returns A ComparisonResult indicating whether the two std::unordered_map<Rule::Id,
/// DiscreteValueRule::DiscreteValue> are equal.
common::ComparisonResult<std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>> IsEqual(
    const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& a,
    const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& b);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Evaluates the equality of two DirectionUsageRule::State::Type.
/// @param a The first DirectionUsageRule::State::Type to compare.
/// @param b The second DirectionUsageRule::State::Type to compare.
/// @returns A ComparisonResult indicating whether the two DirectionUsageRule::State::Type are equal.
common::ComparisonResult<DirectionUsageRule::State::Type> IsEqual(DirectionUsageRule::State::Type a,
                                                                  DirectionUsageRule::State::Type b);

/// Evaluates the equality of two DirectionUsageRule::State::Severity.
/// @param a The first DirectionUsageRule::State::Severity to compare.
/// @param b The second DirectionUsageRule::State::Severity to compare.
/// @returns A ComparisonResult indicating whether the two DirectionUsageRule::State::Severity are equal.
common::ComparisonResult<DirectionUsageRule::State::Severity> IsEqual(DirectionUsageRule::State::Severity a,
                                                                      DirectionUsageRule::State::Severity b);

/// Evaluates the equality of two DirectionUsageRule::State.
/// @param a The first DirectionUsageRule::State to compare.
/// @param b The second DirectionUsageRule::State to compare.
/// @returns A ComparisonResult indicating whether the two DirectionUsageRule::State are equal.
common::ComparisonResult<DirectionUsageRule::State> IsEqual(const DirectionUsageRule::State& a,
                                                            const DirectionUsageRule::State& b);

/// Evaluates the equality of two std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State>.
/// @param a The first std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State> to compare.
/// @param b The second std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State> to compare.
/// @returns A ComparisonResult indicating whether the two std::unordered_map<DirectionUsageRule::State::Id,
/// DirectionUsageRule::State> are equal.
common::ComparisonResult<std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State>> IsEqual(
    const std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State>& a,
    const std::unordered_map<DirectionUsageRule::State::Id, DirectionUsageRule::State>& b);

/// Evaluates the equality of two DirectionUsageRule.
/// @param a The first DirectionUsageRule to compare.
/// @param b The second DirectionUsageRule to compare.
/// @returns A ComparisonResult indicating whether the two DirectionUsageRule are equal.
common::ComparisonResult<DirectionUsageRule> IsEqual(const DirectionUsageRule& a, const DirectionUsageRule& b);

/// Evaluates the equality of two RightOfWayRule::ZoneType.
/// @param a The first RightOfWayRule::ZoneType to compare.
/// @param b The second RightOfWayRule::ZoneType to compare.
/// @returns A ComparisonResult indicating whether the two RightOfWayRule::ZoneType are equal.
common::ComparisonResult<RightOfWayRule::ZoneType> IsEqual(RightOfWayRule::ZoneType a, RightOfWayRule::ZoneType b);

/// Evaluates the equality of two RightOfWayRule::State::Type.
/// @param a The first RightOfWayRule::State::Type to compare.
/// @param b The second RightOfWayRule::State::Type to compare.
/// @returns A ComparisonResult indicating whether the two RightOfWayRule::State::Type are equal.
common::ComparisonResult<RightOfWayRule::State::Type> IsEqual(RightOfWayRule::State::Type a,
                                                              RightOfWayRule::State::Type b);

/// Evaluates the equality of two std::vector<RightOfWayRule::Id>.
/// @param a The first std::vector<RightOfWayRule::Id> to compare.
/// @param b The second std::vector<RightOfWayRule::Id> to compare.
/// @returns A ComparisonResult indicating whether the two std::vector<RightOfWayRule::Id> are equal.
common::ComparisonResult<std::vector<RightOfWayRule::Id>> IsEqual(const std::vector<RightOfWayRule::Id>& a,
                                                                  const std::vector<RightOfWayRule::Id>& b);

/// Evaluates the equality of two RightOfWayRule::State.
/// @param a The first RightOfWayRule::State to compare.
/// @param b The second RightOfWayRule::State to compare.
/// @returns A ComparisonResult indicating whether the two RightOfWayRule::State are equal.
common::ComparisonResult<RightOfWayRule::State> IsEqual(const RightOfWayRule::State& a, const RightOfWayRule::State& b);

/// Evaluates the equality of two std::unordered_map<RightOfWayRule::State::Id, RightOfWayRule::State>.
/// @param a The first std::unordered_map<RightOfWayRule::State::Id, RightOfWayRule::State> to compare.
/// @param b The second std::unordered_map<RightOfWayRule::State::Id, RightOfWayRule::State> to compare.
/// @returns A ComparisonResult indicating whether the two std::unordered_map<RightOfWayRule::State::Id,
/// RightOfWayRule::State> are equal.
common::ComparisonResult<std::unordered_map<RightOfWayRule::State::Id, RightOfWayRule::State>> IsEqual(
    const std::unordered_map<RightOfWayRule::State::Id, RightOfWayRule::State>& a,
    const std::unordered_map<RightOfWayRule::State::Id, RightOfWayRule::State>& b);

/// Evaluates the equality of two RightOfWayRule.
/// @param a The first RightOfWayRule to compare.
/// @param b The second RightOfWayRule to compare.
/// @returns A ComparisonResult indicating whether the two RightOfWayRule are equal.
common::ComparisonResult<RightOfWayRule> IsEqual(const RightOfWayRule& a, const RightOfWayRule& b);

/// Evaluates the equality of two RightOfWayRuleStateProvider::RightOfWayResult.
/// @param a The first RightOfWayRuleStateProvider::RightOfWayResult to compare.
/// @param b The second RightOfWayRuleStateProvider::RightOfWayResult to compare.
/// @returns A ComparisonResult indicating whether the two RightOfWayRuleStateProvider::RightOfWayResult are equal.
common::ComparisonResult<RightOfWayRuleStateProvider::RightOfWayResult> IsEqual(
    const RightOfWayRuleStateProvider::RightOfWayResult& a, const RightOfWayRuleStateProvider::RightOfWayResult& b);

/// Evaluates the equality of two SpeedLimitRule::Severity.
/// @param a The first SpeedLimitRule::Severity to compare.
/// @param b The second SpeedLimitRule::Severity to compare.
/// @returns A ComparisonResult indicating whether the two SpeedLimitRule::Severity are equal.
common::ComparisonResult<SpeedLimitRule::Severity> IsEqual(SpeedLimitRule::Severity a, SpeedLimitRule::Severity b);

/// Evaluates the equality of two SpeedLimitRule.
/// @param a The first SpeedLimitRule to compare.
/// @param b The second SpeedLimitRule to compare.
/// @returns A ComparisonResult indicating whether the two SpeedLimitRule are equal.
common::ComparisonResult<SpeedLimitRule> IsEqual(const SpeedLimitRule& a, const SpeedLimitRule& b);

#pragma GCC diagnostic pop

/// Evaluates the equality of two std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>.
/// @param a The first std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>> to compare.
/// @param b The second std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>> to compare.
/// @returns A ComparisonResult indicating whether the two std::unordered_map<TrafficLight::Id,
/// std::vector<BulbGroup::Id>> are equal.
common::ComparisonResult<std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>> IsEqual(
    const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& a,
    const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& b);

/// Evaluates the equality of two BulbColor.
/// @param a The first BulbColor to compare.
/// @param b The second BulbColor to compare.
/// @returns A ComparisonResult indicating whether the two BulbColor are equal.
common::ComparisonResult<BulbColor> IsEqual(const BulbColor& a, const BulbColor& b);

/// Evaluates the equality of two BulbType.
/// @param a The first BulbType to compare.
/// @param b The second BulbType to compare.
/// @returns A ComparisonResult indicating whether the two BulbType are equal.
common::ComparisonResult<BulbType> IsEqual(const BulbType& a, const BulbType& b);

/// Evaluates the equality of two BulbState.
/// @param a The first BulbState to compare.
/// @param b The second BulbState to compare.
/// @returns A ComparisonResult indicating whether the two BulbState are equal.
common::ComparisonResult<BulbState> IsEqual(const BulbState& a, const BulbState& b);

/// Evaluates the equality of two std::optional<BulbStates>.
/// @param a The first std::optional<BulbStates> to compare.
/// @param b The second std::optional<BulbStates> to compare.
/// @returns A ComparisonResult indicating whether the two std::optional<BulbStates> are equal.
common::ComparisonResult<std::optional<BulbStates>> IsEqual(const std::optional<BulbStates>& a,
                                                            const std::optional<BulbStates>& b);

/// Evaluates the equality of two std::optional<double>.
/// @param a The first std::optional<double> to compare.
/// @param b The second std::optional<double> to compare.
/// @returns A ComparisonResult indicating whether the two std::optional<double> are equal.
common::ComparisonResult<std::optional<double>> IsEqual(const std::optional<double>& a, const std::optional<double>& b);

/// Evaluates the equality of two Bulb::BoundingBox.
/// @param a The first Bulb::BoundingBox to compare.
/// @param b The second Bulb::BoundingBox to compare.
/// @returns A ComparisonResult indicating whether the two Bulb::BoundingBox are equal.
common::ComparisonResult<Bulb::BoundingBox> IsEqual(const Bulb::BoundingBox& a, const Bulb::BoundingBox& b);

/// Evaluates the equality of two Bulb.
/// @param a The first Bulb to compare.
/// @param b The second Bulb to compare.
/// @returns A ComparisonResult indicating whether the two Bulb are equal.
common::ComparisonResult<Bulb> IsEqual(const Bulb* a, const Bulb* b);

/// Evaluates the equality of two std::vector<const Bulb*>.
/// @param a_expression The expression of the first std::vector<const Bulb*> to compare.
/// @param b_expression The expression of the second std::vector<const Bulb*> to compare.
/// @param a The first std::vector<const Bulb*> to compare.
/// @param b The second std::vector<const Bulb*> to compare.
/// @returns A ComparisonResult indicating whether the two std::vector<const Bulb*> are equal.
common::ComparisonResult<std::vector<const Bulb*>> IsEqual(const char* a_expression, const char* b_expression,
                                                           const std::vector<const Bulb*>& a,
                                                           const std::vector<const Bulb*>& b);

/// Evaluates the equality of two BulbGroup.
/// @param a The first BulbGroup to compare.
/// @param b The second BulbGroup to compare.
/// @returns A ComparisonResult indicating whether the two BulbGroup are equal.
common::ComparisonResult<BulbGroup> IsEqual(const BulbGroup* a, const BulbGroup* b);

/// Evaluates the equality of two TrafficLight.
/// @param a The first TrafficLight to compare.
/// @param b The second TrafficLight to compare.
/// @returns A ComparisonResult indicating whether the two TrafficLight are equal.
common::ComparisonResult<TrafficLight> IsEqual(const TrafficLight* a, const TrafficLight* b);

}  // namespace rules
}  // namespace api
}  // namespace maliput
