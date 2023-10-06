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

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/common/compare.h"

namespace maliput {
namespace api {
namespace rules {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
common::ComparisonResult<RuleStates> IsEqual(const RuleStates& a, const RuleStates& b);
#pragma GCC diagnostic pop

common::ComparisonResult<BulbState> IsEqual(const BulbState& a, const BulbState& b);
common::ComparisonResult<std::optional<BulbStates>> IsEqual(const std::optional<BulbStates>& a,
                                                            const std::optional<BulbStates>& b);

common::ComparisonResult<Phase> IsEqual(const Phase& a, const Phase& b);

common::ComparisonResult<PhaseRing::NextPhase> IsEqual(const PhaseRing::NextPhase& a, const PhaseRing::NextPhase& b);

common::ComparisonResult<std::vector<PhaseRing::NextPhase>> IsEqual(const std::vector<PhaseRing::NextPhase>& a,
                                                                    const std::vector<PhaseRing::NextPhase>& b);

common::ComparisonResult<RangeValueRule::Range> IsEqual(const rules::RangeValueRule::Range& a,
                                                        const rules::RangeValueRule::Range& b);
common::ComparisonResult<RangeValueRule> IsEqual(const rules::RangeValueRule& a, const rules::RangeValueRule& b);
common::ComparisonResult<std::vector<rules::RangeValueRule::Range>> IsEqual(
    const std::vector<rules::RangeValueRule::Range>& a, const std::vector<rules::RangeValueRule::Range>& b);

common::ComparisonResult<DiscreteValueRule::DiscreteValue> IsEqual(const DiscreteValueRule::DiscreteValue& a,
                                                                   const DiscreteValueRule::DiscreteValue& b);
common::ComparisonResult<DiscreteValueRule> IsEqual(const DiscreteValueRule& a, const DiscreteValueRule& b);
common::ComparisonResult<std::vector<DiscreteValueRule::DiscreteValue>> IsEqual(
    const std::vector<DiscreteValueRule::DiscreteValue>& a, const std::vector<DiscreteValueRule::DiscreteValue>& b);

common::ComparisonResult<std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>> IsEqual(
    const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& a,
    const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& b);

}  // namespace rules
}  // namespace api
}  // namespace maliput
