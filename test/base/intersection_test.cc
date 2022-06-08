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
#include "maliput/base/intersection.h"

#include <exception>

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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const api::rules::RightOfWayRule::Id rule_id_a{"rule_id"};
  const api::rules::RuleStates rule_states_1_{
      {api::rules::RightOfWayRule::Id(rule_id_a), api::rules::RightOfWayRule::State::Id("GO")}};
  const api::rules::RuleStates rule_states_2_{
      {api::rules::RightOfWayRule::Id(rule_id_a), api::rules::RightOfWayRule::State::Id("STOP")}};
#pragma GCC diagnostic pop

  const api::rules::DiscreteValueRule::DiscreteValue discrete_value_1{
      api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{}, api::rules::Rule::RelatedUniqueIds{}, "Go"};
  const api::rules::DiscreteValueRule::DiscreteValue discrete_value_2{
      api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{}, api::rules::Rule::RelatedUniqueIds{}, "Stop"};
  const api::rules::Rule::Id rule_id_b{"rule_id"};
  const api::rules::DiscreteValueRuleStates discrete_value_rule_states_1_{
      {api::rules::Rule::Id(rule_id_b), discrete_value_1}};
  const api::rules::DiscreteValueRuleStates discrete_value_rule_states_2_{
      {api::rules::Rule::Id(rule_id_b), discrete_value_2}};

  const api::rules::TrafficLight::Id traffic_light{"traffic_light"};
  const api::rules::UniqueBulbId unique_bulb_id{traffic_light, api::rules::BulbGroup::Id("bulb_group"),
                                                api::rules::Bulb::Id("bulb")};
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

  const Intersection::Id kIntersectionId{"intersection"};
};

TEST_F(IntersectionTest, BasicTest) {
  const double kDurationUntil{1};  // Arbitrary.
  ManualPhaseProvider phase_provider;
  Intersection dut(kIntersectionId, ranges_a, dummy_ring_, &phase_provider);
  EXPECT_EQ(dut.id(), kIntersectionId);
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
  EXPECT_EQ(dut.DiscreteValueRuleStates().value().at(rule_id_b), discrete_value_2);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_EQ(dut.RuleStates().value().at(rule_id_a), api::rules::RightOfWayRule::State::Id("STOP"));
#pragma GCC diagnostic pop
  dut.SetPhase(dummy_phase_1_.id(), dummy_phase_2_.id(), kDurationUntil);
  EXPECT_EQ(dut.bulb_states().value().at(unique_bulb_id), api::rules::BulbState::kOff);
  EXPECT_EQ(dut.DiscreteValueRuleStates().value().at(rule_id_b), discrete_value_1);
  EXPECT_THROW(dut.SetPhase(dummy_phase_1_.id(), std::nullopt, kDurationUntil), std::exception);
}

TEST_F(IntersectionTest, IncludesByGeoPos) {
  const api::InertialPosition kGeoPos{11.8, 89., 1.};
  ManualPhaseProvider phase_provider;
  {
    const Intersection dut(kIntersectionId, ranges_a, dummy_ring_, &phase_provider);
    EXPECT_TRUE(dut.Includes(api::InertialPosition{11.8, 89., 1.}, road_geometry.get()));
  }
  {
    const Intersection dut(kIntersectionId, ranges_b, dummy_ring_, &phase_provider);
    EXPECT_FALSE(dut.Includes(api::InertialPosition{11.8, 89., 1.}, road_geometry.get()));
  }
  {
    const Intersection dut(kIntersectionId, ranges_c, dummy_ring_, &phase_provider);
    EXPECT_THROW(dut.Includes(api::InertialPosition{11.8, 89., 1.}, road_geometry.get()), common::assertion_error);
  }
  {
    const Intersection dut(kIntersectionId, ranges_c, dummy_ring_, &phase_provider);
    EXPECT_THROW(dut.Includes(api::InertialPosition{11.8, 89., 1.}, nullptr), common::assertion_error);
  }
}

TEST_F(IntersectionTest, IncludesByTrafficLightId) {
  const api::rules::TrafficLight::Id not_included_traffic_light{"not_included_traffic_light"};
  ManualPhaseProvider phase_provider;
  Intersection dut(kIntersectionId, ranges_a, dummy_ring_, &phase_provider);
  phase_provider.AddPhaseRing(dummy_ring_.id(), dummy_phase_1_.id());

  EXPECT_TRUE(dut.Includes(traffic_light));
  EXPECT_FALSE(dut.Includes(not_included_traffic_light));
}

TEST_F(IntersectionTest, IncludesByDiscreteValueRuleId) {
  const api::rules::Rule::Id not_included_rule_id{"not_included_rule_id"};
  ManualPhaseProvider phase_provider;
  Intersection dut(kIntersectionId, ranges_a, dummy_ring_, &phase_provider);
  phase_provider.AddPhaseRing(dummy_ring_.id(), dummy_phase_1_.id());

  EXPECT_TRUE(dut.Includes(rule_id_b));
  EXPECT_FALSE(dut.Includes(not_included_rule_id));
}

TEST_F(IntersectionTest, IncludesByRightOfWayRuleId) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const api::rules::RightOfWayRule::Id not_included_rule_id{"not_included_rule_id"};
  ManualPhaseProvider phase_provider;
  Intersection dut(kIntersectionId, ranges_a, dummy_ring_, &phase_provider);
  phase_provider.AddPhaseRing(dummy_ring_.id(), dummy_phase_1_.id());

  EXPECT_TRUE(dut.Includes(rule_id_a));
  EXPECT_FALSE(dut.Includes(not_included_rule_id));
#pragma GCC diagnostic pop
}

}  // namespace
}  // namespace test
}  // namespace maliput
