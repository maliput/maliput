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
#include "maliput/api/road_network.h"

#include <exception>
#include <utility>

#include <gtest/gtest.h>

#include "maliput/api/intersection.h"
#include "maliput/geometry_base/road_geometry.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/mock_geometry.h"

namespace maliput {
namespace api {
namespace test {
namespace {

using rules::DirectionUsageRule;
using rules::DiscreteValueRuleStateProvider;
using rules::PhaseProvider;
using rules::PhaseRingBook;
using rules::RangeValueRuleStateProvider;
using rules::RightOfWayRuleStateProvider;
using rules::RoadRulebook;
using rules::Rule;
using rules::RuleRegistry;
using rules::SpeedLimitRule;
using rules::TrafficLightBook;

class RoadNetworkTest : public ::testing::Test {
 protected:
  const double linear_tolerance{1.};
  const double angular_tolerance{1.};
  const double scale_length{1.};
  const math::Vector3 inertial_to_backend_frame_translation{0., 0., 0.};

  virtual void SetUp() {
    road_geometry_ = test::CreateRoadGeometry();
    road_rulebook_ = test::CreateRoadRulebook();
    intersection_book_ = test::CreateIntersectionBook();
    traffic_light_book_ = test::CreateTrafficLightBook();
    phase_ring_book_ = test::CreatePhaseRingBook();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    right_of_way_rule_state_provider_ = test::CreateRightOfWayRuleStateProvider();
#pragma GCC diagnostic pop
    phase_provider_ = test::CreatePhaseProvider();
    rule_registry_ = test::CreateRuleRegistry();
    discrete_value_rule_state_provider_ = test::CreateDiscreteValueRuleStateProvider();
    range_value_rule_state_provider_ = test::CreateRangeValueRuleStateProvider();

    road_geometry_ptr_ = road_geometry_.get();
    road_rulebook_ptr_ = road_rulebook_.get();
    traffic_light_book_ptr_ = traffic_light_book_.get();
    intersection_book_ptr_ = intersection_book_.get();
    phase_ring_book_ptr_ = phase_ring_book_.get();
    right_of_way_rule_state_provider_ptr_ = right_of_way_rule_state_provider_.get();
    phase_provider_ptr_ = phase_provider_.get();
    rule_registry_ptr_ = rule_registry_.get();
    discrete_value_rule_state_provider_ptr_ = discrete_value_rule_state_provider_.get();
    range_value_rule_state_provider_ptr_ = range_value_rule_state_provider_.get();
  }

  std::unique_ptr<RoadGeometry> road_geometry_;
  std::unique_ptr<RoadRulebook> road_rulebook_;
  std::unique_ptr<TrafficLightBook> traffic_light_book_;
  std::unique_ptr<IntersectionBook> intersection_book_;
  std::unique_ptr<PhaseRingBook> phase_ring_book_;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::unique_ptr<RightOfWayRuleStateProvider> right_of_way_rule_state_provider_;
#pragma GCC diagnostic pop
  std::unique_ptr<PhaseProvider> phase_provider_;
  std::unique_ptr<RuleRegistry> rule_registry_;
  std::unique_ptr<DiscreteValueRuleStateProvider> discrete_value_rule_state_provider_;
  std::unique_ptr<RangeValueRuleStateProvider> range_value_rule_state_provider_;

  RoadGeometry* road_geometry_ptr_{};
  RoadRulebook* road_rulebook_ptr_{};
  TrafficLightBook* traffic_light_book_ptr_{};
  IntersectionBook* intersection_book_ptr_{};
  PhaseRingBook* phase_ring_book_ptr_{};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  RightOfWayRuleStateProvider* right_of_way_rule_state_provider_ptr_{};
#pragma GCC diagnostic pop
  PhaseProvider* phase_provider_ptr_{};
  RuleRegistry* rule_registry_ptr_{};
  DiscreteValueRuleStateProvider* discrete_value_rule_state_provider_ptr_{};
  RangeValueRuleStateProvider* range_value_rule_state_provider_ptr_{};
};

TEST_F(RoadNetworkTest, MissingParameters) {
  EXPECT_THROW(
      RoadNetwork(nullptr, std::move(road_rulebook_), std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(right_of_way_rule_state_provider_), std::move(phase_provider_),
                  std::move(rule_registry_), std::move(discrete_value_rule_state_provider_),
                  std::move(range_value_rule_state_provider_)),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), nullptr, std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(right_of_way_rule_state_provider_), std::move(phase_provider_),
                  std::move(rule_registry_), std::move(discrete_value_rule_state_provider_),
                  std::move(range_value_rule_state_provider_)),
      std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), nullptr, std::move(intersection_book_),
                           std::move(phase_ring_book_), std::move(right_of_way_rule_state_provider_),
                           std::move(phase_provider_), std::move(rule_registry_),
                           std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           nullptr, std::move(phase_ring_book_), std::move(right_of_way_rule_state_provider_),
                           std::move(phase_provider_), std::move(rule_registry_),
                           std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), nullptr, std::move(right_of_way_rule_state_provider_),
                           std::move(phase_provider_), std::move(rule_registry_),
                           std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), std::move(phase_ring_book_), nullptr,
                           std::move(phase_provider_), std::move(rule_registry_),
                           std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), std::move(phase_ring_book_),
                           std::move(right_of_way_rule_state_provider_), nullptr, std::move(rule_registry_),
                           std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), std::move(phase_ring_book_),
                           std::move(right_of_way_rule_state_provider_), std::move(phase_provider_), nullptr,
                           std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), std::move(phase_ring_book_),
                           std::move(right_of_way_rule_state_provider_), std::move(phase_provider_),
                           std::move(rule_registry_), nullptr, std::move(range_value_rule_state_provider_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), std::move(phase_ring_book_),
                           std::move(right_of_way_rule_state_provider_), std::move(phase_provider_),
                           std::move(rule_registry_), std::move(discrete_value_rule_state_provider_), nullptr),
               std::exception);
}

TEST_F(RoadNetworkTest, InstantiateAndUseAccessors) {
  RoadNetwork dut(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                  std::move(intersection_book_), std::move(phase_ring_book_),
                  std::move(right_of_way_rule_state_provider_), std::move(phase_provider_), std::move(rule_registry_),
                  std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_));

  EXPECT_EQ(dut.road_geometry(), road_geometry_ptr_);
  EXPECT_EQ(dut.rulebook(), road_rulebook_ptr_);
  EXPECT_EQ(dut.traffic_light_book(), traffic_light_book_ptr_);
  EXPECT_EQ(dut.intersection_book(), intersection_book_ptr_);
  EXPECT_EQ(dut.phase_ring_book(), phase_ring_book_ptr_);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_EQ(dut.right_of_way_rule_state_provider(), right_of_way_rule_state_provider_ptr_);
#pragma GCC diagnostic pop
  EXPECT_EQ(dut.phase_provider(), phase_provider_ptr_);
  EXPECT_EQ(dut.rule_registry(), rule_registry_ptr_);
  EXPECT_EQ(dut.discrete_value_rule_state_provider(), discrete_value_rule_state_provider_ptr_);
  EXPECT_EQ(dut.range_value_rule_state_provider(), range_value_rule_state_provider_ptr_);
}

TEST_F(RoadNetworkTest, TestMemberMethodAccess) {
  RoadNetwork dut(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                  std::move(intersection_book_), std::move(phase_ring_book_),
                  std::move(right_of_way_rule_state_provider_), std::move(phase_provider_), std::move(rule_registry_),
                  std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_));

  auto intersection = dut.intersection_book()->GetIntersection(Intersection::Id("Mock"));
  EXPECT_NE(intersection, nullptr);
  intersection->SetPhase(rules::Phase::Id("Mock"));

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  dut.rulebook()->GetRule(rules::RightOfWayRule::Id("Mock"));
  dut.right_of_way_rule_state_provider()->GetState(rules::RightOfWayRule::Id("Mock"));
#pragma GCC diagnostic pop
  dut.phase_ring_book()->GetPhaseRing(rules::PhaseRing::Id("Mock"));
  dut.traffic_light_book()->GetTrafficLight(rules::TrafficLight::Id("Mock"));
  dut.phase_provider()->GetPhase(rules::PhaseRing::Id("Mock"));
  dut.rule_registry()->GetPossibleStatesOfRuleType(rules::Rule::TypeId("Mock"));
  dut.discrete_value_rule_state_provider()->GetState(rules::Rule::Id("Mock"));
  dut.range_value_rule_state_provider()->GetState(rules::Rule::Id("Mock"));
}

TEST_F(RoadNetworkTest, Contains) {
  auto mock_road_geometry = std::make_unique<geometry_base::test::MockRoadGeometry>(
      api::RoadGeometryId{"mock_road_geometry"}, linear_tolerance, angular_tolerance, scale_length,
      inertial_to_backend_frame_translation);
  auto mock_lane = std::make_unique<geometry_base::test::MockLane>(api::LaneId{"mock_lane"});
  auto mock_segment = std::make_unique<geometry_base::test::MockSegment>(api::SegmentId{"mock_segment"});
  auto mock_junction = std::make_unique<geometry_base::test::MockJunction>(api::JunctionId{"mock_junction"});

  mock_segment->AddLane(std::move(mock_lane));
  mock_junction->AddSegment(std::move(mock_segment));
  mock_road_geometry->AddJunction(std::move(mock_junction));

  RoadNetwork dut(std::move(mock_road_geometry), std::move(road_rulebook_), std::move(traffic_light_book_),
                  std::move(intersection_book_), std::move(phase_ring_book_),
                  std::move(right_of_way_rule_state_provider_), std::move(phase_provider_), std::move(rule_registry_),
                  std::move(discrete_value_rule_state_provider_), std::move(range_value_rule_state_provider_));

  ASSERT_TRUE(dut.Contains(api::LaneId("mock_lane")));
  ASSERT_FALSE(dut.Contains(api::LaneId("false_lane")));
}

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace maliput
