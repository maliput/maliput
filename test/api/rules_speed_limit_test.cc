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
/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/speed_limit_rule.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "maliput/api/regions.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/regions_test_utilities.h"
#include "maliput/test_utilities/rules_speed_limit_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

const LaneSRange kZone(LaneId("the_lane"), SRange(13., 15.));

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
GTEST_TEST(SpeedLimitRuleTest, Construction) {
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone, SpeedLimitRule::Severity::kStrict, 33., 77.));
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone, SpeedLimitRule::Severity::kStrict, 0., 77.));
  EXPECT_NO_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone, SpeedLimitRule::Severity::kStrict, 0., 0.));

  // Min must not be greater than max.
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone, SpeedLimitRule::Severity::kStrict, 90., 77.),
               maliput::common::assertion_error);
  // Negative limits are not allowed.
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone, SpeedLimitRule::Severity::kStrict, -8., 77.),
               maliput::common::assertion_error);
  EXPECT_THROW(SpeedLimitRule(SpeedLimitRule::Id("some_id"), kZone, SpeedLimitRule::Severity::kStrict, -8., -6.),
               maliput::common::assertion_error);
}

GTEST_TEST(SpeedLimitRuleTest, Accessors) {
  const SpeedLimitRule dut(SpeedLimitRule::Id("dut_id"), kZone, SpeedLimitRule::Severity::kStrict, 5., 8.);
  EXPECT_EQ(dut.id(), SpeedLimitRule::Id("dut_id"));
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.zone(), kZone));
  EXPECT_EQ(dut.severity(), SpeedLimitRule::Severity::kStrict);
  EXPECT_EQ(dut.min(), 5.);
  EXPECT_EQ(dut.max(), 8.);

  // Test another severity and the effectively-no-minimum case as well.
  const SpeedLimitRule dut2(SpeedLimitRule::Id("dut_id"), kZone, SpeedLimitRule::Severity::kAdvisory, 0., 8.);
  EXPECT_EQ(dut2.severity(), SpeedLimitRule::Severity::kAdvisory);
  EXPECT_EQ(dut2.min(), 0.);
}

GTEST_TEST(SpeedLimitRuleTest, Copying) {
  const SpeedLimitRule source(SpeedLimitRule::Id("dut_id"), kZone, SpeedLimitRule::Severity::kStrict, 5., 8.);

  const SpeedLimitRule dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(SpeedLimitRuleTest, Assignment) {
  const SpeedLimitRule source(SpeedLimitRule::Id("dut_id"), kZone, SpeedLimitRule::Severity::kStrict, 0., 8.);
  SpeedLimitRule dut(SpeedLimitRule::Id("other_id"), kZone, SpeedLimitRule::Severity::kAdvisory, 70., 90.);

  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}
#pragma GCC diagnostic pop

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
