#include "maliput/api/rules/range_value_rule_state_provider.h"

#include <gtest/gtest.h>

#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Mock class to evaluate RangeValueRuleStateProvider interface.
class MockRangeValueRuleStateProvider : public RangeValueRuleStateProvider {
 public:
  static const double kTolerance;
  static const Rule::Id kRuleId;
  static const Rule::TypeId kRuleType;
  static const RoadPosition kRoadPosition;
  static RangeValueRule::Range MakeCurrentRange();
  static RangeValueRule::Range MakeNextRange();

 private:
  std::optional<StateResult> DoGetState(const Rule::Id& id) const override {
    if (id == kRuleId) {
      return StateResult{MakeCurrentRange(), StateResult::Next{MakeNextRange(), {123.456} /* duration */}};
    }
    return {};
  }
  std::optional<StateResult> DoGetState(const RoadPosition& road_position, const Rule::TypeId& rule_type,
                                        double tolerance) const override {
    if (road_position.lane == kRoadPosition.lane && road_position.pos.srh() == kRoadPosition.pos.srh() &&
        rule_type == kRuleType && tolerance == kTolerance) {
      return StateResult{MakeCurrentRange(), StateResult::Next{MakeNextRange(), {123.456} /* duration */}};
    }
    return {};
  }
};

const Rule::Id MockRangeValueRuleStateProvider::kRuleId{"RuleId"};
const double MockRangeValueRuleStateProvider::kTolerance{1e-3};
// Using nullptr lane for the sake of the test.
const RoadPosition MockRangeValueRuleStateProvider::kRoadPosition{nullptr, LanePosition{0., 0., 0.}};
const Rule::TypeId MockRangeValueRuleStateProvider::kRuleType{"My-Rule-Type"};

RangeValueRule::Range MockRangeValueRuleStateProvider::MakeCurrentRange() {
  return RangeValueRule::Range{Rule::State::kStrict,
                               maliput::api::test::CreateEmptyRelatedRules(),
                               maliput::api::test::CreateEmptyRelatedUniqueIds(),
                               "current_range_description",
                               56. /* min */,
                               78. /* max*/};
}

RangeValueRule::Range MockRangeValueRuleStateProvider::MakeNextRange() {
  return RangeValueRule::Range{Rule::State::kStrict,
                               maliput::api::test::CreateEmptyRelatedRules(),
                               maliput::api::test::CreateEmptyRelatedUniqueIds(),
                               "next_range_description",
                               12. /* min */,
                               4. /* max*/};
}

GTEST_TEST(RangeValueRuleStateProviderTest, GetStateById) {
  const MockRangeValueRuleStateProvider dut;
  const std::optional<RangeValueRuleStateProvider::StateResult> result =
      dut.GetState(MockRangeValueRuleStateProvider::kRuleId);

  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, MockRangeValueRuleStateProvider::MakeCurrentRange()));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, MockRangeValueRuleStateProvider::MakeNextRange()));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), 123.456);

  EXPECT_FALSE(dut.GetState(Rule::Id("UnregisteredRule")).has_value());
}

GTEST_TEST(RangeValueRuleStateProviderTest, GetStateByRoadPositionAndType) {
  const MockRangeValueRuleStateProvider dut;
  const std::optional<RangeValueRuleStateProvider::StateResult> result =
      dut.GetState(MockRangeValueRuleStateProvider::kRoadPosition, MockRangeValueRuleStateProvider::kRuleType,
                   MockRangeValueRuleStateProvider::kTolerance);

  EXPECT_TRUE(result.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->state, MockRangeValueRuleStateProvider::MakeCurrentRange()));
  EXPECT_TRUE(result->next.has_value());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(result->next->state, MockRangeValueRuleStateProvider::MakeNextRange()));
  EXPECT_TRUE(result->next->duration_until.has_value());
  EXPECT_EQ(result->next->duration_until.value(), 123.456);

  EXPECT_FALSE(dut.GetState(RoadPosition{nullptr, LanePosition{100., 0., 0.}},
                            MockRangeValueRuleStateProvider::kRuleType, MockRangeValueRuleStateProvider::kTolerance)
                   .has_value());
  EXPECT_FALSE(dut.GetState(MockRangeValueRuleStateProvider::kRoadPosition, Rule::TypeId("Unkown Rule Type"),
                            MockRangeValueRuleStateProvider::kTolerance)
                   .has_value());
  EXPECT_FALSE(
      dut.GetState(MockRangeValueRuleStateProvider::kRoadPosition, MockRangeValueRuleStateProvider::kRuleType, 800)
          .has_value());
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
