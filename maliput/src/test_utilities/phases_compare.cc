#include "maliput/test_utilities/phases_compare.h"

#include "drake/common/unused.h"

#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/test_utilities/traffic_lights_compare.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const RuleStates& a,
                                   const RuleStates& b) {
  drake::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  for (const auto& rule_state : a) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(b.at(rule_state.first), rule_state.second));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const DiscreteValueRuleStates& a,
                                   const DiscreteValueRuleStates& b) {
  drake::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  for (const auto& discrete_value_rule_state : a) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(b.at(discrete_value_rule_state.first), discrete_value_rule_state.second));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::optional<BulbStates>& a, const std::optional<BulbStates>& b) {
  drake::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.has_value(), b.has_value()));
  if (a.has_value()) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->size(), b->size()));
    for (const auto& bulb_state : *a) {
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(b->at(bulb_state.first), bulb_state.second));
    }
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const Phase& a, const Phase& b) {
  drake::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.rule_states(), b.rule_states()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.discrete_value_rule_states(), b.discrete_value_rule_states()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.bulb_states(), b.bulb_states()));
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const PhaseRing::NextPhase& a,
                                   const PhaseRing::NextPhase& b) {
  drake::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id, b.id));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.duration_until, b.duration_until));
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::vector<PhaseRing::NextPhase>& a,
                                   const std::vector<PhaseRing::NextPhase>& b) {
  drake::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  if (a.size() == b.size()) {
    for (size_t i = 0; i < a.size(); ++i) {
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.at(i), b.at(i)));
    }
  }
  return c.result();
}

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
