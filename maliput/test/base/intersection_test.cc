#include "maliput/base/intersection.h"

#include <exception>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"

namespace maliput {
namespace {

class IntersectionTest : public ::testing::Test {
 public:
  IntersectionTest()
      : dummy_phase_1_(api::rules::Phase::Id("dummy_phase_1"), rule_states_1_),
        dummy_phase_2_(api::rules::Phase::Id("dummy_phase_2"), rule_states_2_),
        dummy_ring_(api::rules::PhaseRing::Id("dummy_ring"), {dummy_phase_1_, dummy_phase_2_}) {}

  const api::rules::RuleStates rule_states_1_{
      {api::rules::RightOfWayRule::Id("dummy_rule"), api::rules::RightOfWayRule::State::Id("GO")}};
  const api::rules::RuleStates rule_states_2_{
      {api::rules::RightOfWayRule::Id("dummy_rule"), api::rules::RightOfWayRule::State::Id("STOP")}};

  const api::rules::Phase dummy_phase_1_;
  const api::rules::Phase dummy_phase_2_;

  const api::rules::PhaseRing dummy_ring_;

  const std::vector<api::LaneSRange> ranges_{api::LaneSRange(api::LaneId("road A"), api::SRange(0, 100))};
};

TEST_F(IntersectionTest, BasicTest) {
  const double kDurationUntil{1};  // Arbitrary.
  const Intersection::Id intersection_id("foo");
  ManualPhaseProvider phase_provider;
  Intersection dut(intersection_id, ranges_, dummy_ring_, &phase_provider);
  EXPECT_EQ(dut.id(), intersection_id);
  EXPECT_EQ(dut.Phase(), drake::nullopt);
  phase_provider.AddPhaseRing(dummy_ring_.id(), dummy_phase_1_.id());
  EXPECT_EQ(dut.Phase()->id, dummy_phase_1_.id());
  dut.SetPhase(dummy_phase_2_.id(), dummy_phase_1_.id(), kDurationUntil);
  EXPECT_EQ(dut.Phase()->id, dummy_phase_2_.id());
  EXPECT_TRUE(dut.Phase()->next.has_value());
  EXPECT_EQ(dut.Phase()->next->id, dummy_phase_1_.id());
  EXPECT_TRUE(dut.Phase()->next->duration_until.has_value());
  EXPECT_EQ(dut.Phase()->next->duration_until.value(), kDurationUntil);
  EXPECT_EQ(dut.region().size(), ranges_.size());
  EXPECT_EQ(dut.region().at(0).lane_id(), ranges_.at(0).lane_id());
  EXPECT_EQ(dut.ring_id(), dummy_ring_.id());
  EXPECT_EQ(dut.bulb_states(), drake::nullopt);
  EXPECT_THROW(dut.SetPhase(dummy_phase_1_.id(), drake::nullopt, kDurationUntil), std::exception);
}

}  // namespace
}  // namespace maliput
