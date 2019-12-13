#include "maliput/base/intersection.h"

#include <exception>

#include <optional>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace test {
namespace {

class IntersectionTest : public ::testing::Test {
 public:
  IntersectionTest()
      : dummy_phase_1_(api::rules::Phase::Id("phase_1"), rule_states_1_, discrete_value_rule_states_1_, bulb_states_1_),
        dummy_phase_2_(api::rules::Phase::Id("phase_2"), rule_states_2_, discrete_value_rule_states_2_, bulb_states_2_),
        dummy_ring_(api::rules::PhaseRing::Id("ring"), {dummy_phase_1_, dummy_phase_2_}) {}

  const api::rules::RuleStates rule_states_1_{
      {api::rules::RightOfWayRule::Id("rule_id"), api::rules::RightOfWayRule::State::Id("GO")}};
  const api::rules::RuleStates rule_states_2_{
      {api::rules::RightOfWayRule::Id("rule_id"), api::rules::RightOfWayRule::State::Id("STOP")}};

  const api::rules::DiscreteValueRule::DiscreteValue discrete_value_1 = api::rules::MakeDiscreteValue(
      api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{}, api::rules::Rule::RelatedUniqueIds{}, "Go");
  const api::rules::DiscreteValueRule::DiscreteValue discrete_value_2 = api::rules::MakeDiscreteValue(
      api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{}, api::rules::Rule::RelatedUniqueIds{}, "Stop");

  const api::rules::DiscreteValueRuleStates discrete_value_rule_states_1_{
      {api::rules::Rule::Id("rule_id"), discrete_value_1}};
  const api::rules::DiscreteValueRuleStates discrete_value_rule_states_2_{
      {api::rules::Rule::Id("rule_id"), discrete_value_2}};

  const api::rules::UniqueBulbId unique_bulb_id{api::rules::TrafficLight::Id("traffic_light"),
                                                api::rules::BulbGroup::Id("bulb_group"), api::rules::Bulb::Id("bulb")};
  const std::optional<api::rules::BulbStates> bulb_states_1_ =
      std::optional<api::rules::BulbStates>({{unique_bulb_id, api::rules::BulbState::kOff}});

  const std::optional<api::rules::BulbStates> bulb_states_2_ =
      std::optional<api::rules::BulbStates>({{unique_bulb_id, api::rules::BulbState::kOn}});

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
  EXPECT_EQ(dut.Phase(), std::nullopt);
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
  EXPECT_EQ(dut.bulb_states().value().at(unique_bulb_id), api::rules::BulbState::kOn);
  EXPECT_EQ(dut.DiscreteValueRuleStates().value().at(api::rules::Rule::Id("rule_id")), discrete_value_2);
  dut.SetPhase(dummy_phase_1_.id(), dummy_phase_2_.id(), kDurationUntil);
  EXPECT_EQ(dut.bulb_states().value().at(unique_bulb_id), api::rules::BulbState::kOff);
  EXPECT_EQ(dut.DiscreteValueRuleStates().value().at(api::rules::Rule::Id("rule_id")), discrete_value_1);
  EXPECT_THROW(dut.SetPhase(dummy_phase_1_.id(), std::nullopt, kDurationUntil), std::exception);
}

TEST_F(IntersectionTest, IncludesByGeoPos) {
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

TEST_F(IntersectionTest, IncludesByTrafficLightId) {
  const Intersection::Id kIntersectionId("intersection");
  const double kDurationUntil{1};
  ManualPhaseProvider phase_provider;
  Intersection dut(kIntersectionId, ranges_a, dummy_ring_, &phase_provider);
  phase_provider.AddPhaseRing(dummy_ring_.id(), dummy_phase_1_.id());

  EXPECT_TRUE(dut.Includes(api::rules::TrafficLight::Id("traffic_light")));
  EXPECT_FALSE(dut.Includes(api::rules::TrafficLight::Id("false_traffic_light")));
}

TEST_F(IntersectionTest, IncludesByRuleId) {
  const Intersection::Id kIntersectionId("intersection");
  const double kDurationUntil{1};
  ManualPhaseProvider phase_provider;
  Intersection dut(kIntersectionId, ranges_a, dummy_ring_, &phase_provider);
  phase_provider.AddPhaseRing(dummy_ring_.id(), dummy_phase_1_.id());

  EXPECT_TRUE(dut.Includes(api::rules::DiscreteValueRule::Id("rule_id")));
  EXPECT_FALSE(dut.Includes(api::rules::DiscreteValueRule::Id("false_rule_id")));
}

}  // namespace
}  // namespace test
}  // namespace maliput
