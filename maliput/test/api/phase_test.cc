/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/phase.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <vector>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/phases_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

class PhaseTest : public ::testing::Test {
 protected:
  PhaseTest()
      : id_("test_id"),
        rule_states_{{RightOfWayRule::Id("northbound-forward"), RightOfWayRule::State::Id("GO")},
                     {RightOfWayRule::Id("southbound-left-turn"), RightOfWayRule::State::Id("STOP")}},
        discrete_value_rule_states_{{Rule::Id("RightOfWayRuleType/northbound-forward"),
                                     MakeDiscreteValue(Rule::State::kStrict, {} /* related_rules */, "Go")},
                                    {Rule::Id("RightOfWayRuleType/southbound-left-turn"),
                                     MakeDiscreteValue(Rule::State::kStrict, {} /* related_rules */, "Stop")}},
        bulb_states_{
            {{{TrafficLight::Id("major-intersection"), BulbGroup::Id("northbound"), Bulb::Id("forward-green")},
              BulbState::kOn},
             {{TrafficLight::Id("major-intersection"), BulbGroup::Id("northbound"), Bulb::Id("forward-red")},
              BulbState::kOff},
             {{TrafficLight::Id("major-intersection"), BulbGroup::Id("southbound"), Bulb::Id("left-turn-green")},
              BulbState::kOff},
             {{TrafficLight::Id("major-intersection"), BulbGroup::Id("southbound"), Bulb::Id("left-turn-red")},
              BulbState::kOn}}},
        phase_{id_, rule_states_, discrete_value_rule_states_, bulb_states_} {}

  const Phase::Id id_;
  const RuleStates rule_states_;
  const DiscreteValueRuleStates discrete_value_rule_states_;
  const drake::optional<BulbStates> bulb_states_;
  const Phase phase_;
};

TEST_F(PhaseTest, Accessors) {
  for (const drake::optional<BulbStates>& bulb_states :
       std::vector<drake::optional<BulbStates>>{drake::nullopt, bulb_states_}) {
    Phase dut(id_, rule_states_, discrete_value_rule_states_, bulb_states);
    EXPECT_EQ(dut.id(), id_);
    EXPECT_EQ(dut.rule_states(), rule_states_);
    EXPECT_EQ(dut.discrete_value_rule_states(), discrete_value_rule_states_);
    EXPECT_EQ(dut.bulb_states(), bulb_states);
  }
}

TEST_F(PhaseTest, Copying) {
  const Phase dut(phase_);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, phase_));
}

TEST_F(PhaseTest, Assignment) {
  Phase dut(Phase::Id("other_dut_id"), RuleStates(), discrete_value_rule_states_);
  dut = phase_;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, phase_));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
