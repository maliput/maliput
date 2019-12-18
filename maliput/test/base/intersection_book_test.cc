#include "maliput/base/intersection_book.h"

#include <exception>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/rule.h"
#include "maliput/base/intersection.h"
#include "maliput/base/manual_phase_provider.h"

namespace maliput {
namespace {

using api::LaneSRange;

// Returns an arbitrary Phase.
api::rules::Phase CreatePhase(const api::rules::Phase::Id& id) {
  return api::rules::Phase{
      id,
      {{api::rules::RightOfWayRule::Id("rule_a"), api::rules::RightOfWayRule::State::Id("GO")},
       {api::rules::RightOfWayRule::Id("rule_b"), api::rules::RightOfWayRule::State::Id("STOP")}},
      {{api::rules::Rule::Id("RightOfWayRuleType/rule_a"),
        api::rules::DiscreteValueRule::DiscreteValue{api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{},
                                                     api::rules::Rule::RelatedUniqueIds{}, "Go"}},
       {api::rules::Rule::Id("RightOfWayRuleType/rule_b"),
        api::rules::DiscreteValueRule::DiscreteValue{api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{},
                                                     api::rules::Rule::RelatedUniqueIds{}, "Stop"}}},
      {{{{api::rules::TrafficLight::Id("traffic_light_a"), api::rules::BulbGroup::Id("bulb_group_a"),
          api::rules::Bulb::Id("rule_a_green")},
         api::rules::BulbState::kOn},
        {{api::rules::TrafficLight::Id("traffic_light_b"), api::rules::BulbGroup::Id("bulb_group_b"),
          api::rules::Bulb::Id("rule_b_red")},
         api::rules::BulbState::kOn}}}};
}

GTEST_TEST(IntersectionBookTest, BasicTest) {
  ManualPhaseProvider phase_provider;
  const api::rules::PhaseRing phase_ring(api::rules::PhaseRing::Id("phase_ring_id"),
                                         {CreatePhase(api::rules::Phase::Id("phase_id"))}, std::nullopt);
  const Intersection::Id id("my intersection");
  const std::vector<api::LaneSRange> region;
  auto intersection = std::make_unique<Intersection>(id, region, phase_ring, &phase_provider);
  const Intersection* intersection_ptr = intersection.get();
  IntersectionBook dut;
  EXPECT_THROW(dut.AddIntersection(nullptr), std::exception);
  dut.AddIntersection(std::move(intersection));
  EXPECT_EQ(dut.GetIntersection(Intersection::Id("unknown")), nullptr);
  EXPECT_EQ(dut.GetIntersection(id), intersection_ptr);
}

// Returns true when `intersections` has an Intersection whose ID is `expected_intersection_id`.
bool HasIntersectionId(const std::vector<maliput::api::Intersection*>& intersections,
                       const Intersection::Id& expected_intersection_id) {
  return std::find_if(intersections.begin(), intersections.end(),
                      [expected_intersection_id](const maliput::api::Intersection* intersection) {
                        return intersection->id() == expected_intersection_id;
                      }) != intersections.end();
}

GTEST_TEST(IntersectionBookTest, FindIntersections) {
  const double tolerance = 1e-3;
  ManualPhaseProvider phase_provider;
  const api::rules::PhaseRing phase_ring(api::rules::PhaseRing::Id("phase_ring_id"),
                                         {CreatePhase(api::rules::Phase::Id("phase_id"))}, std::nullopt);
  const Intersection::Id kIntersectionIdA("intersection_a");
  const std::vector<api::LaneSRange> region_a{{
      LaneSRange{api::LaneId{"lane_a_1"}, api::SRange{0., 20.}},
      LaneSRange{api::LaneId{"lane_a_2"}, api::SRange{0., 20.}},
      LaneSRange{api::LaneId{"lane_a_3"}, api::SRange{0., 20.}},
  }};
  const Intersection::Id kIntersectionIdB("intersection_b");
  const std::vector<api::LaneSRange> region_b{{
      LaneSRange{api::LaneId{"lane_b_1"}, api::SRange{21., 40.}},
      LaneSRange{api::LaneId{"lane_b_2"}, api::SRange{21., 40.}},
      LaneSRange{api::LaneId{"lane_b_3"}, api::SRange{21., 40.}},
  }};
  const Intersection::Id kIntersectionIdC("intersection_c");
  const std::vector<api::LaneSRange> region_c{{
      LaneSRange{api::LaneId{"lane_a_1"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_b_1"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_a_2"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_b_2"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_c_1"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_c_2"}, api::SRange{0., 100.}},
  }};
  IntersectionBook intersection_book;
  auto intersection_a = std::make_unique<Intersection>(kIntersectionIdA, region_a, phase_ring, &phase_provider);
  auto intersection_b = std::make_unique<Intersection>(kIntersectionIdB, region_b, phase_ring, &phase_provider);
  auto intersection_c = std::make_unique<Intersection>(kIntersectionIdC, region_c, phase_ring, &phase_provider);

  intersection_book.AddIntersection(std::move(intersection_a));
  intersection_book.AddIntersection(std::move(intersection_b));
  intersection_book.AddIntersection(std::move(intersection_c));

  EXPECT_EQ(intersection_book.FindIntersections({}, tolerance).size(), 0);
  {
    const std::vector<maliput::api::Intersection*> dut{intersection_book.FindIntersections(region_a, tolerance)};
    EXPECT_EQ(dut.size(), 2);
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdA));
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdC));
  }
  {
    const std::vector<maliput::api::Intersection*> dut(intersection_book.FindIntersections(region_c, tolerance));
    EXPECT_EQ(dut.size(), 3);
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdA));
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdB));
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdC));
  }
  {
    const std::vector<maliput::api::Intersection*> dut(
        intersection_book.FindIntersections({LaneSRange{api::LaneId{"lane_c_1"}, api::SRange{0., 10.}}}, tolerance));
    EXPECT_EQ(dut.size(), 1);
    EXPECT_EQ(dut[0]->id(), kIntersectionIdC);
  }
}

}  // namespace
}  // namespace maliput
