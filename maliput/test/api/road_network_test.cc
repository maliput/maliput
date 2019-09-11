#include "maliput/api/road_network.h"

#include <exception>
#include <utility>

#include <gtest/gtest.h>

#include "maliput/api/intersection.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace api {
namespace {

using rules::DirectionUsageRule;
using rules::PhaseProvider;
using rules::PhaseRingBook;
using rules::RightOfWayRuleStateProvider;
using rules::RoadRulebook;
using rules::Rule;
using rules::RuleRegistry;
using rules::SpeedLimitRule;
using rules::TrafficLightBook;

class RoadNetworkTest : public ::testing::Test {
 protected:
  virtual void SetUp() {
    road_geometry_ = test::CreateRoadGeometry();
    road_rulebook_ = test::CreateRoadRulebook();
    intersection_book_ = test::CreateIntersectionBook();
    traffic_light_book_ = test::CreateTrafficLightBook();
    phase_ring_book_ = test::CreatePhaseRingBook();
    right_of_way_rule_state_provider_ = test::CreateRightOfWayRuleStateProvider();
    phase_provider_ = test::CreatePhaseProvider();
    rule_registry_ = test::CreateRuleRegistry();

    road_geometry_ptr_ = road_geometry_.get();
    road_rulebook_ptr_ = road_rulebook_.get();
    traffic_light_book_ptr_ = traffic_light_book_.get();
    intersection_book_ptr_ = intersection_book_.get();
    phase_ring_book_ptr_ = phase_ring_book_.get();
    right_of_way_rule_state_provider_ptr_ = right_of_way_rule_state_provider_.get();
    phase_provider_ptr_ = phase_provider_.get();
    rule_registry_ptr_ = rule_registry_.get();
  }

  std::unique_ptr<RoadGeometry> road_geometry_;
  std::unique_ptr<RoadRulebook> road_rulebook_;
  std::unique_ptr<TrafficLightBook> traffic_light_book_;
  std::unique_ptr<IntersectionBook> intersection_book_;
  std::unique_ptr<PhaseRingBook> phase_ring_book_;
  std::unique_ptr<RightOfWayRuleStateProvider> right_of_way_rule_state_provider_;
  std::unique_ptr<PhaseProvider> phase_provider_;
  std::unique_ptr<RuleRegistry> rule_registry_;

  RoadGeometry* road_geometry_ptr_{};
  RoadRulebook* road_rulebook_ptr_{};
  TrafficLightBook* traffic_light_book_ptr_{};
  IntersectionBook* intersection_book_ptr_{};
  PhaseRingBook* phase_ring_book_ptr_{};
  RightOfWayRuleStateProvider* right_of_way_rule_state_provider_ptr_{};
  PhaseProvider* phase_provider_ptr_{};
  RuleRegistry* rule_registry_ptr_{};
};

TEST_F(RoadNetworkTest, MissingParameters) {
  EXPECT_THROW(
      RoadNetwork(nullptr, std::move(road_rulebook_), std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(right_of_way_rule_state_provider_), std::move(phase_provider_),
                  std::move(rule_registry_)),
      std::exception);
  EXPECT_THROW(
      RoadNetwork(std::move(road_geometry_), nullptr, std::move(traffic_light_book_), std::move(intersection_book_),
                  std::move(phase_ring_book_), std::move(right_of_way_rule_state_provider_), std::move(phase_provider_),
                  std::move(rule_registry_)),
      std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), nullptr, std::move(intersection_book_),
                           std::move(phase_ring_book_), std::move(right_of_way_rule_state_provider_),
                           std::move(phase_provider_), std::move(rule_registry_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           nullptr, std::move(phase_ring_book_), std::move(right_of_way_rule_state_provider_),
                           std::move(phase_provider_), std::move(rule_registry_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), nullptr, std::move(right_of_way_rule_state_provider_),
                           std::move(phase_provider_), std::move(rule_registry_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), std::move(phase_ring_book_), nullptr,
                           std::move(phase_provider_), std::move(rule_registry_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), std::move(phase_ring_book_),
                           std::move(right_of_way_rule_state_provider_), nullptr, std::move(rule_registry_)),
               std::exception);
  EXPECT_THROW(RoadNetwork(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                           std::move(intersection_book_), std::move(phase_ring_book_),
                           std::move(right_of_way_rule_state_provider_), std::move(phase_provider_), nullptr),
               std::exception);
}

TEST_F(RoadNetworkTest, InstantiateAndUseAccessors) {
  RoadNetwork dut(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                  std::move(intersection_book_), std::move(phase_ring_book_),
                  std::move(right_of_way_rule_state_provider_), std::move(phase_provider_), std::move(rule_registry_));

  EXPECT_EQ(dut.road_geometry(), road_geometry_ptr_);
  EXPECT_EQ(dut.rulebook(), road_rulebook_ptr_);
  EXPECT_EQ(dut.traffic_light_book(), traffic_light_book_ptr_);
  EXPECT_EQ(dut.intersection_book(), intersection_book_ptr_);
  EXPECT_EQ(dut.phase_ring_book(), phase_ring_book_ptr_);
  EXPECT_EQ(dut.right_of_way_rule_state_provider(), right_of_way_rule_state_provider_ptr_);
  EXPECT_EQ(dut.phase_provider(), phase_provider_ptr_);
  EXPECT_EQ(dut.rule_registry(), rule_registry_ptr_);
}

TEST_F(RoadNetworkTest, TestMemberMethodAccess) {
  RoadNetwork dut(std::move(road_geometry_), std::move(road_rulebook_), std::move(traffic_light_book_),
                  std::move(intersection_book_), std::move(phase_ring_book_),
                  std::move(right_of_way_rule_state_provider_), std::move(phase_provider_), std::move(rule_registry_));

  auto intersection = dut.intersection_book()->GetIntersection(Intersection::Id("Mock"));
  EXPECT_NE(intersection, nullptr);
  intersection->SetPhase(rules::Phase::Id("Mock"));

  dut.rulebook()->GetRule(rules::RightOfWayRule::Id("Mock"));
  dut.traffic_light_book()->GetTrafficLight(rules::TrafficLight::Id("Mock"));
  dut.phase_ring_book()->GetPhaseRing(rules::PhaseRing::Id("Mock"));
  dut.right_of_way_rule_state_provider()->GetState(rules::RightOfWayRule::Id("Mock"));
  dut.phase_provider()->GetPhase(rules::PhaseRing::Id("Mock"));
  dut.rule_registry()->GetPossibleStatesOfRuleType(rules::Rule::TypeId("Mock"));
}

}  // namespace
}  // namespace api
}  // namespace maliput
