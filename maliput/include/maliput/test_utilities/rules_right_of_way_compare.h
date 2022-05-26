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

#include <gtest/gtest.h>

#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.

/// Predicate-formatter which tests equality of RightOfWayRule::ZoneType.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   rules::RightOfWayRule::ZoneType a, rules::RightOfWayRule::ZoneType b);

// TODO(maddog@tri.global)  This should be replaced by a generic predicate
//                          which handles anything with operator==.
/// Predicate-formatter which tests equality of RightOfWayRule::State::Type.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   rules::RightOfWayRule::State::Type a, rules::RightOfWayRule::State::Type b);

/// Predicate-formatter which tests equality of RightOfWayRule::State.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const rules::RightOfWayRule::State& a, const rules::RightOfWayRule::State& b);
#pragma GCC diagnostic pop

/// Predicate-formatter which tests equality of
/// RightOfWayRule::RelatedBulbGroups which expands to std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& a,
                                   const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& b);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
/// Predicate-formatter which tests equality of RightOfWayRule.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const rules::RightOfWayRule& a,
                                   const rules::RightOfWayRule& b);

/// Predicate-formatter which tests equality of
/// RuleStateProvider::RightOfWayResult.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const rules::RightOfWayRuleStateProvider::RightOfWayResult& a,
                                   const rules::RightOfWayRuleStateProvider::RightOfWayResult& b);
#pragma GCC diagnostic pop

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
