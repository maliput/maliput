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
#include "maliput/test_utilities/phases_compare.h"

#include "maliput/common/maliput_unused.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/test_utilities/traffic_lights_compare.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const RuleStates& a,
                                   const RuleStates& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  for (const auto& rule_state : a) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(b.at(rule_state.first), rule_state.second));
  }
  return c.result();
}
#pragma GCC diagnostic pop

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& a,
                                   const std::unordered_map<Rule::Id, DiscreteValueRule::DiscreteValue>& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  for (const auto& discrete_value_rule_state : a) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(b.at(discrete_value_rule_state.first), discrete_value_rule_state.second));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::optional<BulbStates>& a, const std::optional<BulbStates>& b) {
  maliput::common::unused(a_expression, b_expression);
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
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.rule_states(), b.rule_states()));
#pragma GCC diagnostic pop
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.discrete_value_rule_states(), b.discrete_value_rule_states()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.bulb_states(), b.bulb_states()));
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const PhaseRing::NextPhase& a,
                                   const PhaseRing::NextPhase& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id, b.id));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.duration_until, b.duration_until));
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::vector<PhaseRing::NextPhase>& a,
                                   const std::vector<PhaseRing::NextPhase>& b) {
  maliput::common::unused(a_expression, b_expression);
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
