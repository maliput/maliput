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
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "maliput/test_utilities/rules_right_of_way_compare.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
class RightOfWayRuleStateProviderTest : public ::testing::Test {
 protected:
  const RightOfWayRule::Id kExistingId{"aye"};
  const RightOfWayRule::Id kNonExistingId{"nay"};

  const RightOfWayRule::State::Id kCurrentStateId{"state"};
  const RightOfWayRule::State::Id kNextStateId{"next"};
  const double kDurationUntil{99.};

  class MockStateProvider : public RightOfWayRuleStateProvider {
   public:
    explicit MockStateProvider(const RightOfWayRuleStateProviderTest* fixture) : fixture_(fixture) {}

   private:
    std::optional<RightOfWayResult> DoGetState(const RightOfWayRule::Id& id) const final {
      if (id == fixture_->kExistingId) {
        return RightOfWayResult{fixture_->kCurrentStateId,
                                RightOfWayResult::Next{fixture_->kNextStateId, fixture_->kDurationUntil}};
      }
      return std::nullopt;
    }

    const RightOfWayRuleStateProviderTest* const fixture_;
  };
};

TEST_F(RightOfWayRuleStateProviderTest, ExerciseInterface) {
  using Result = RightOfWayRuleStateProvider::RightOfWayResult;

  const MockStateProvider dut(this);

  EXPECT_TRUE(dut.GetState(kExistingId).has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetState(kExistingId).value(),
                               (Result{kCurrentStateId, Result::Next{kNextStateId, kDurationUntil}})));

  EXPECT_FALSE(dut.GetState(kNonExistingId).has_value());
}
#pragma GCC diagnostic pop

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
