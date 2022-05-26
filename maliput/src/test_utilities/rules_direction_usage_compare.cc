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
#include "maliput/test_utilities/rules_direction_usage_compare.h"

#include <gtest/gtest.h>

#include "maliput/common/maliput_unused.h"
#include "maliput/test_utilities/regions_test_utilities.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   rules::DirectionUsageRule::State::Type a, rules::DirectionUsageRule::State::Type b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   rules::DirectionUsageRule::State::Severity a,
                                   rules::DirectionUsageRule::State::Severity b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const rules::DirectionUsageRule::State& a,
                                   const rules::DirectionUsageRule::State& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.severity(), b.severity()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type(), b.type()));
  return c.result();
}

::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const std::unordered_map<rules::DirectionUsageRule::State::Id, rules::DirectionUsageRule::State>& a,
    const std::unordered_map<rules::DirectionUsageRule::State::Id, rules::DirectionUsageRule::State>& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const std::unordered_map<rules::DirectionUsageRule::State::Id, rules::DirectionUsageRule::State>& largest =
      (a.size() < b.size()) ? b : a;

  for (const auto& pair : largest) {
    const rules::DirectionUsageRule::State::Id& key = pair.first;
    auto a_it = a.find(key);
    auto b_it = b.find(key);
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL((a_it != a.cend()), true));
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL((b_it != b.cend()), true));
    if ((a_it != a.cend()) && (b_it != b.cend())) {
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a_it->second, b_it->second));
    }
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const rules::DirectionUsageRule& a, const rules::DirectionUsageRule& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_REGIONS_IS_EQUAL(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.states(), b.states()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.is_static(), b.is_static()));
  if (a.is_static() && b.is_static()) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.static_state(), b.static_state()));
  }
  return c.result();
}
#pragma GCC diagnostic pop

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
