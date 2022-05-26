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

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/regions.h"
#include "maliput/common/maliput_unused.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace test {

/// Returns an AssertionResult which is successful if `e1` equals `e2`
/// according to the `IsEqual()` predicate-formatter function.  The
/// literal expressions for `e1` and `e2` will be provided to `IsEqual()`.
#define MALIPUT_REGIONS_IS_EQUAL(e1, e2) ::maliput::api::test::IsEqual(#e1, #e2, e1, e2)

// TODO(agalbachicar)  This should be replaced by a generic predicate
//                     which handles anything with operator==.

/// Predicate-formatter which tests equality of SRange.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const SRange& a,
                                          const SRange& b) {
  maliput::common::unused(a_expression, b_expression);
  rules::test::AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.s0(), b.s0()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.s1(), b.s1()));
  return c.result();
}

/// Predicate-formatter which tests equality of LaneSRange.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const LaneSRange& a,
                                          const LaneSRange& b) {
  maliput::common::unused(a_expression, b_expression);
  rules::test::AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.lane_id(), b.lane_id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_REGIONS_IS_EQUAL(a.s_range(), b.s_range()));
  return c.result();
}

/// Predicate-formatter which tests equality of std::vector<LaneSRange>.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                          const std::vector<LaneSRange>& a, const std::vector<LaneSRange>& b) {
  maliput::common::unused(a_expression, b_expression);
  rules::test::AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_REGIONS_IS_EQUAL(a[i], b[i]));
  }
  return c.result();
}

/// Predicate-formatter which tests equality of LaneSRoute.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const LaneSRoute& a,
                                          const LaneSRoute& b) {
  maliput::common::unused(a_expression, b_expression);
  rules::test::AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_REGIONS_IS_EQUAL(a.ranges(), b.ranges()));
  return c.result();
}

}  // namespace test
}  // namespace api
}  // namespace maliput
