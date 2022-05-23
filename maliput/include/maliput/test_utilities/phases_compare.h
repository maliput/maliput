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
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/traffic_lights.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Predicate-formatter which tests equality of RuleStates.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const RuleStates& a,
                                   const RuleStates& b);
#pragma GCC diagnostic pop

/// Predicate-formatter which tests equality of DiscreteValueRuleStates,
/// which expands to std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& a,
                                   const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& b);

/// Predicate-formatter which tests equality of std::optional<BulbStates>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::optional<BulbStates>& a, const std::optional<BulbStates>& b);

/// Predicate-formatter which tests equality of Phase.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const Phase& a, const Phase& b);

/// Predicate-formatter which tests equality of PhaseRing::NextPhase.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const PhaseRing::NextPhase& a,
                                   const PhaseRing::NextPhase& b);

/// Predicate-formatter which tests equality of
/// std::vector<PhaseRing::NextPhase>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::vector<PhaseRing::NextPhase>& a,
                                   const std::vector<PhaseRing::NextPhase>& b);

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
