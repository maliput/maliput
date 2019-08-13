#pragma once

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/rules/rule.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

// TODO(agalbachicar)  This should be replaced by a generic predicate
//                     which handles anything with operator==.

/// Predicate-formatter which tests equality of a vector of Rule::Ids.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const std::vector<rules::Rule::Id>& a,
                                   const std::vector<rules::Rule::Id>& b);

/// Predicate-formatter which tests equality of RangeValueRule::Range.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RangeValueRule::Range& a,
                                   const rules::RangeValueRule::Range& b);

/// Predicate-formatter which tests equality of a vector of
/// RangeValueRule::Ranges.
::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const std::vector<rules::RangeValueRule::Range>& a,
    const std::vector<rules::RangeValueRule::Range>& b);


/// Predicate-formatter which tests equality of RangeValueRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RangeValueRule& a,
                                   const rules::RangeValueRule& b);

/// Predicate-formatter which tests equality of a string.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const std::string& a,
                                   const std::string& b);

/// Predicate-formatter which tests equality of a vector of strings.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const std::vector<std::string>& a,
                                   const std::vector<std::string>& b);

/// Predicate-formatter which tests equality of DiscreteValueRule.
::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::DiscreteValueRule& a,
                                   const rules::DiscreteValueRule& b);



}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
