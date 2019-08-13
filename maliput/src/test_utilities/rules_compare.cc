#include "maliput/test_utilities/rules_compare.h"

#include <algorithm>

#include "drake/common/unused.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const std::vector<rules::Rule::Id>& a,
                                   const std::vector<rules::Rule::Id>& b) {
  drake::unused(a_expression);
  drake::unused(b_expression);

  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.at(i), b.at(i)));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const std::string& a,
                                   const std::string& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}


::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const std::vector<std::string>& a,
                                   const std::vector<std::string>& b) {
  drake::unused(a_expression);
  drake::unused(b_expression);

  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.at(i), b.at(i)));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::DiscreteValueRule& a,
                                   const rules::DiscreteValueRule& b) {
  drake::unused(a_expression);
  drake::unused(b_expression);

  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type_id(), b.type_id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.related_rules(), b.related_rules()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.values(), b.values()));
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RangeValueRule::Range& a,
                                   const rules::RangeValueRule::Range& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const std::vector<rules::RangeValueRule::Range>& a,
    const std::vector<rules::RangeValueRule::Range>& b) {
  drake::unused(a_expression);
  drake::unused(b_expression);

  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.at(i), b.at(i)));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression,
                                   const char* b_expression,
                                   const rules::RangeValueRule& a,
                                   const rules::RangeValueRule& b) {
  drake::unused(a_expression);
  drake::unused(b_expression);

  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type_id(), b.type_id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.related_rules(), b.related_rules()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.ranges(), b.ranges()));
  return c.result();
}


}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
