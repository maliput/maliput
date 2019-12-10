#include "maliput/base/intersection.h"

#include <exception>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace test {
namespace {

class IntersectionTest : public ::testing::Test {
 public:
  IntersectionTest()
      : dummy_phase_1_(api::rules::Phase::Id("dummy_phase_1"), rule_states_1_, discrete_value_rule_states_1_),
        dummy_phase_2_(api::rules::Phase::Id("dummy_phase_2"), rule_states_2_, discrete_value_rule_states_2_),
        dummy_ring_(api::rules::PhaseRing::Id("dummy_ring"), {dummy_phase_1_, dummy_phase_2_}) {}

  const api::rules::RuleStates rule_states_1_{
      {api::rules::RightOfWayRule::Id("dummy_rule"), api::rules::RightOfWayRule::State::Id("GO")}};
  const api::rules::RuleStates rule_states_2_{
      {api::rules::RightOfWayRule::Id("dummy_rule"), api::rules::RightOfWayRule::State::Id("STOP")}};

  const api::rules::DiscreteValueRuleStates discrete_value_rule_states_1_{
      {api::rules::Rule::Id("dummy_rule"),
       api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{},
                                     api::rules::Rule::RelatedUniqueIds{}, "Go")}};
  const api::rules::DiscreteValueRuleStates discrete_value_rule_states_2_{
      {api::rules::Rule::Id("dummy_rule"),
       api::rules::MakeDiscreteValue(api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{},
                                     api::rules::Rule::RelatedUniqueIds{}, "Stop")}};

  const api::rules::Phase dummy_phase_1_;
  const api::rules::Phase dummy_phase_2_;

  const api::rules::PhaseRing dummy_ring_;

  std::unique_ptr<api::RoadGeometry> road_geometry = api::test::CreateTwoLanesRoadGeometry();
  const std::vector<api::LaneSRange> ranges_a{api::LaneSRange(api::LaneId("lane_a"), api::SRange(0, 100))};
  const std::vector<api::LaneSRange> ranges_b{api::LaneSRange(api::LaneId("lane_b"), api::SRange(0, 100))};
  const std::vector<api::LaneSRange> ranges_c{api::LaneSRange(api::LaneId("lane_c"), api::SRange(0, 100))};
};

TEST_F(IntersectionTest, BasicTest) {
  const double kDurationUntil{1};  // Arbitrary.
  const Intersection::Id intersection_id("foo");
  ManualPhaseProvider phase_provider;
  Intersection dut(intersection_id, ranges_a, dummy_ring_, &phase_provider);
  EXPECT_EQ(dut.id(), intersection_id);
  EXPECT_EQ(dut.Phase(), drake::nullopt);
  phase_provider.AddPhaseRing(dummy_ring_.id(), dummy_phase_1_.id());
  EXPECT_EQ(dut.Phase()->state, dummy_phase_1_.id());
  dut.SetPhase(dummy_phase_2_.id(), dummy_phase_1_.id(), kDurationUntil);
  EXPECT_EQ(dut.Phase()->state, dummy_phase_2_.id());
  EXPECT_TRUE(dut.Phase()->next.has_value());
  EXPECT_EQ(dut.Phase()->next->state, dummy_phase_1_.id());
  EXPECT_TRUE(dut.Phase()->next->duration_until.has_value());
  EXPECT_EQ(dut.Phase()->next->duration_until.value(), kDurationUntil);
  EXPECT_EQ(dut.region().size(), ranges_a.size());
  EXPECT_EQ(dut.region().at(0).lane_id(), ranges_a.at(0).lane_id());
  EXPECT_EQ(dut.ring_id(), dummy_ring_.id());
  EXPECT_EQ(dut.bulb_states(), drake::nullopt);
  EXPECT_THROW(dut.SetPhase(dummy_phase_1_.id(), drake::nullopt, kDurationUntil), std::exception);
}

TEST_F(IntersectionTest, Includes) {
  const Intersection::Id kIntersectionId("intersection");
  const api::GeoPosition kGeoPos{11.8, 89., 1.};
  ManualPhaseProvider phase_provider;
  {
    const Intersection dut(kIntersectionId, ranges_a, dummy_ring_, &phase_provider);
    EXPECT_TRUE(dut.Includes(api::GeoPosition{11.8, 89., 1.}, road_geometry.get()));
  }
  {
    const Intersection dut(kIntersectionId, ranges_b, dummy_ring_, &phase_provider);
    EXPECT_FALSE(dut.Includes(api::GeoPosition{11.8, 89., 1.}, road_geometry.get()));
  }
  {
    const Intersection dut(kIntersectionId, ranges_c, dummy_ring_, &phase_provider);
    EXPECT_THROW(dut.Includes(api::GeoPosition{11.8, 89., 1.}, road_geometry.get()), common::assertion_error);
  }
  {
    const Intersection dut(kIntersectionId, ranges_c, dummy_ring_, &phase_provider);
    EXPECT_THROW(dut.Includes(api::GeoPosition{11.8, 89., 1.}, nullptr), common::assertion_error);
  }
}

}  // namespace
}  // namespace test
}  // namespace maliput
