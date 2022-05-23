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
#include "maliput/test_utilities/rules_right_of_way_compare.h"

#include <algorithm>

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
                                   rules::RightOfWayRule::ZoneType a, rules::RightOfWayRule::ZoneType b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   rules::RightOfWayRule::State::Type a, rules::RightOfWayRule::State::Type b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

// TODO(maddog@tri.global)  Make a generic template for vector<T>.
::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const std::vector<rules::RightOfWayRule::Id>& a,
                                   const std::vector<rules::RightOfWayRule::Id>& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const int smallest = std::min(a.size(), b.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a[i], b[i]));
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const rules::RightOfWayRule::State& a, const rules::RightOfWayRule::State& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.type(), b.type()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.yield_to(), b.yield_to()));
  return c.result();
}

::testing::AssertionResult IsEqual(
    const char* a_expression, const char* b_expression,
    const std::unordered_map<rules::RightOfWayRule::State::Id, rules::RightOfWayRule::State>& a,
    const std::unordered_map<rules::RightOfWayRule::State::Id, rules::RightOfWayRule::State>& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  const std::unordered_map<rules::RightOfWayRule::State::Id, rules::RightOfWayRule::State>& largest =
      (a.size() < b.size()) ? b : a;

  for (const auto& pair : largest) {
    const rules::RightOfWayRule::State::Id& key = pair.first;
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
                                   const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& a,
                                   const std::unordered_map<TrafficLight::Id, std::vector<BulbGroup::Id>>& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;

  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.size(), b.size()));
  if (a.size() == b.size()) {
    for (const auto& traffic_light_bulb_group : a) {
      const auto& b_it = b.find(traffic_light_bulb_group.first);
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL((b_it != b.cend()), true));
      if (b_it == b.cend()) {
        break;
      }
      for (const BulbGroup::Id& bulb_group_id : traffic_light_bulb_group.second) {
        const auto& b_bulb_group_id_it = std::find(b_it->second.cbegin(), b_it->second.cend(), bulb_group_id);
        MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL((b_bulb_group_id_it != b_it->second.cend()), true));
        if (b_bulb_group_id_it == b_it->second.cend()) {
          break;
        }
      }
    }
  }
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const rules::RightOfWayRule& a,
                                   const rules::RightOfWayRule& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.id(), b.id()));
  MALIPUT_ADD_RESULT(c, MALIPUT_REGIONS_IS_EQUAL(a.zone(), b.zone()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.zone_type(), b.zone_type()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.states(), b.states()));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.is_static(), b.is_static()));
  if (a.is_static() && b.is_static()) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.static_state(), b.static_state()));
  } else {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.states(), b.states()));
  }
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.related_bulb_groups(), b.related_bulb_groups()));
  return c.result();
}

::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                   const rules::RightOfWayRuleStateProvider::RightOfWayResult& a,
                                   const rules::RightOfWayRuleStateProvider::RightOfWayResult& b) {
  maliput::common::unused(a_expression, b_expression);
  AssertionResultCollector c;
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.state, b.state));
  MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.next.has_value(), b.next.has_value()));
  if (a.next.has_value() && b.next.has_value()) {
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.next->state, b.next->state));
    MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.next->duration_until.has_value(), b.next->duration_until.has_value()));
    if (a.next->duration_until.has_value() && b.next->duration_until.has_value()) {
      MALIPUT_ADD_RESULT(c, MALIPUT_IS_EQUAL(a.next->duration_until.value(), b.next->duration_until.value()));
    }
  }
  return c.result();
}
#pragma GCC diagnostic pop

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
