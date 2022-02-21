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
