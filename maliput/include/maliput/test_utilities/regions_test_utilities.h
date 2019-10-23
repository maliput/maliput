#pragma once

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/unused.h"

#include "maliput/api/regions.h"
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
  drake::unused(a_expression, b_expression);
  rules::test::AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.s0(), b.s0()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.s1(), b.s1()));
  return c.result();
}

/// Predicate-formatter which tests equality of LaneSRange.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const LaneSRange& a,
                                          const LaneSRange& b) {
  drake::unused(a_expression, b_expression);
  rules::test::AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.lane_id(), b.lane_id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_REGIONS_IS_EQUAL(a.s_range(), b.s_range()));
  return c.result();
}

/// Predicate-formatter which tests equality of std::vector<LaneSRange>.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                          const std::vector<LaneSRange>& a, const std::vector<LaneSRange>& b) {
  drake::unused(a_expression, b_expression);
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
  drake::unused(a_expression, b_expression);
  rules::test::AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_REGIONS_IS_EQUAL(a.ranges(), b.ranges()));
  return c.result();
}

}  // namespace test
}  // namespace api
}  // namespace maliput
