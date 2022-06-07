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
#include "maliput/test_utilities/traffic_lights_compare.h"

#include <algorithm>

#include "maliput/common/maliput_unused.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const BulbColor& a,
                                   const BulbColor& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const BulbType& a,
                                   const BulbType& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const BulbState& a,
                                   const BulbState& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const std::optional<double>& a,
                                   const std::optional<double>& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const Bulb::BoundingBox& a,
                                   const Bulb::BoundingBox& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  for (int i = 0; i < 3; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.p_BMin[i], b.p_BMin[i]));
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.p_BMax[i], b.p_BMax[i]));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const Bulb* a, const Bulb* b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->id(), b->id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->position_bulb_group(), b->position_bulb_group()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->orientation_bulb_group(), b->orientation_bulb_group()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->color(), b->color()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->type(), b->type()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->arrow_orientation_rad(), b->arrow_orientation_rad()));
  const std::vector<BulbState>& a_states = a->states();
  const std::vector<BulbState>& b_states = b->states();
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a_states.size(), b_states.size()));
  int smallest = std::min(a_states.size(), b_states.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a_states[i], b_states[i]));
  }
  MALIPUT_IS_EQUAL(a->bounding_box(), b->bounding_box());
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::vector<const Bulb*>& a, const std::vector<const Bulb*>& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.at(i), b.at(i)));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const BulbGroup* a,
                                   const BulbGroup* b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->id(), b->id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->position_traffic_light(), b->position_traffic_light()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->orientation_traffic_light(), b->orientation_traffic_light()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->bulbs(), b->bulbs()));
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const TrafficLight* a,
                                   const TrafficLight* b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->id(), b->id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->position_road_network(), b->position_road_network()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a->orientation_road_network(), b->orientation_road_network()));
  const std::vector<const BulbGroup*> bulb_groups_a = a->bulb_groups();
  const std::vector<const BulbGroup*> bulb_groups_b = b->bulb_groups();
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(bulb_groups_a.size(), bulb_groups_b.size()));
  const int smallest = std::min(bulb_groups_a.size(), bulb_groups_b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(bulb_groups_a.at(i), bulb_groups_b.at(i)));
  }
  return c.result();
}

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
