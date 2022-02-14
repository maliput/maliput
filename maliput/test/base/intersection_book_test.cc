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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Returns an arbitrary Phase.
// 'id' value is included in the api::rules::Rule::Id and the api::rules::Rule::TrafficLight::Id of the
// api::rules::Phase only to allow generating different phases. The name does not represent a real usecase.
api::rules::Phase CreatePhase(const api::rules::Phase::Id& id) {
  const std::string phase_id = id.string();
  return api::rules::Phase{
      id,
      {{api::rules::RightOfWayRule::Id("rule_a/" + phase_id), api::rules::RightOfWayRule::State::Id("GO")},
       {api::rules::RightOfWayRule::Id("rule_b/" + phase_id), api::rules::RightOfWayRule::State::Id("STOP")}},
      {{api::rules::Rule::Id("RightOfWayRuleType/rule_a/" + phase_id),
        api::rules::DiscreteValueRule::DiscreteValue{api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{},
                                                     api::rules::Rule::RelatedUniqueIds{}, "Go"}},
       {api::rules::Rule::Id("RightOfWayRuleType/rule_b/" + phase_id),
        api::rules::DiscreteValueRule::DiscreteValue{api::rules::Rule::State::kStrict, api::rules::Rule::RelatedRules{},
                                                     api::rules::Rule::RelatedUniqueIds{}, "Stop"}}},
      {{{{api::rules::TrafficLight::Id("traffic_light_a/" + phase_id), api::rules::BulbGroup::Id("bulb_group_a"),
          api::rules::Bulb::Id("rule_a_green")},
         api::rules::BulbState::kOn},
        {{api::rules::TrafficLight::Id("traffic_light_b/" + phase_id), api::rules::BulbGroup::Id("bulb_group_b"),
          api::rules::Bulb::Id("rule_b_red")},
         api::rules::BulbState::kOn}}}};
}
#pragma GCC diagnostic pop

GTEST_TEST(IntersectionBook, BasicTest) {
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
bool HasIntersectionId(const std::vector<api::Intersection*>& intersections,
                       const Intersection::Id& expected_intersection_id) {
  return std::find_if(intersections.begin(), intersections.end(),
                      [expected_intersection_id](const maliput::api::Intersection* intersection) {
                        return intersection->id() == expected_intersection_id;
                      }) != intersections.end();
}

// Provides an IntersectionBook populated with three Intersections.
class IntersectionBookTest : public ::testing::Test {
 public:
  const api::rules::Phase kPhase1 = CreatePhase(api::rules::Phase::Id("phase_id_1"));
  const api::rules::Phase kPhase2 = CreatePhase(api::rules::Phase::Id("phase_id_2"));
  const api::rules::Phase kPhase3 = CreatePhase(api::rules::Phase::Id("phase_id_3"));
  const api::rules::PhaseRing kPhaseRing1{api::rules::PhaseRing::Id("phase_ring_id_1"), {kPhase1}, std::nullopt};
  const api::rules::PhaseRing kPhaseRing2{api::rules::PhaseRing::Id("phase_ring_id_2"), {kPhase2}, std::nullopt};
  const api::rules::PhaseRing kPhaseRing3{api::rules::PhaseRing::Id("phase_ring_id_3"), {kPhase3}, std::nullopt};
  const Intersection::Id kIntersectionIdA{"intersection_a"};
  const std::vector<api::LaneSRange> region_a{{
      LaneSRange{api::LaneId{"lane_a_1"}, api::SRange{0., 20.}},
      LaneSRange{api::LaneId{"lane_a_2"}, api::SRange{0., 20.}},
      LaneSRange{api::LaneId{"lane_a_3"}, api::SRange{0., 20.}},
  }};
  const Intersection::Id kIntersectionIdB{"intersection_b"};
  const std::vector<api::LaneSRange> region_b{{
      LaneSRange{api::LaneId{"lane_b_1"}, api::SRange{21., 40.}},
      LaneSRange{api::LaneId{"lane_b_2"}, api::SRange{21., 40.}},
      LaneSRange{api::LaneId{"lane_b_3"}, api::SRange{21., 40.}},
  }};
  const Intersection::Id kIntersectionIdC{"intersection_c"};
  const std::vector<api::LaneSRange> region_c{{
      LaneSRange{api::LaneId{"lane_a_1"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_b_1"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_a_2"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_b_2"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_c_1"}, api::SRange{0., 100.}},
      LaneSRange{api::LaneId{"lane_c_2"}, api::SRange{0., 100.}},
  }};
  // TrafficLight::Id constants includes phase_id in its name to match with CreatePhase() function.
  // The name does not represent a real usecase.
  const api::rules::TrafficLight::Id kTrafficLightIdAPhase1{"traffic_light_a/phase_id_1"};
  const api::rules::TrafficLight::Id kTrafficLightIdBPhase2{"traffic_light_b/phase_id_2"};
  const api::rules::TrafficLight::Id kTrafficLightIdAPhase3{"traffic_light_a/phase_id_3"};
  const api::rules::DiscreteValueRule::Id kDiscreteValueRuleIdAPhase1{"RightOfWayRuleType/rule_a/phase_id_1"};
  const api::rules::DiscreteValueRule::Id kDiscreteValueRuleIdBPhase2{"RightOfWayRuleType/rule_b/phase_id_2"};
  const api::rules::DiscreteValueRule::Id kDiscreteValueRuleIdAPhase3{"RightOfWayRuleType/rule_a/phase_id_3"};
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const api::rules::RightOfWayRule::Id kRightOfWayRuleIdAPhase1{"rule_a/phase_id_1"};
  const api::rules::RightOfWayRule::Id kRightOfWayRuleIdBPhase2{"rule_b/phase_id_2"};
  const api::rules::RightOfWayRule::Id kRightOfWayRuleIdAPhase3{"rule_a/phase_id_3"};
#pragma GCC diagnostic pop
  const double kTolerance = 1e-3;

  IntersectionBookTest() {
    phase_provider_.AddPhaseRing(kPhaseRing1.id(), kPhase1.id());
    phase_provider_.AddPhaseRing(kPhaseRing2.id(), kPhase2.id());
    phase_provider_.AddPhaseRing(kPhaseRing3.id(), kPhase3.id());
    auto intersection_a = std::make_unique<Intersection>(kIntersectionIdA, region_a, kPhaseRing1, &phase_provider_);
    auto intersection_b = std::make_unique<Intersection>(kIntersectionIdB, region_b, kPhaseRing2, &phase_provider_);
    auto intersection_c = std::make_unique<Intersection>(kIntersectionIdC, region_c, kPhaseRing3, &phase_provider_);
    intersection_book_.AddIntersection(std::move(intersection_a));
    intersection_book_.AddIntersection(std::move(intersection_b));
    intersection_book_.AddIntersection(std::move(intersection_c));
  }

  ManualPhaseProvider phase_provider_;
  IntersectionBook intersection_book_;
};

TEST_F(IntersectionBookTest, FindIntersections) {
  EXPECT_EQ(static_cast<int>(intersection_book_.FindIntersections({}, kTolerance).size()), 0);
  {
    const std::vector<maliput::api::Intersection*> dut{intersection_book_.FindIntersections(region_a, kTolerance)};
    EXPECT_EQ(static_cast<int>(dut.size()), 2);
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdA));
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdC));
  }
  {
    const std::vector<maliput::api::Intersection*> dut(intersection_book_.FindIntersections(region_c, kTolerance));
    EXPECT_EQ(static_cast<int>(dut.size()), 3);
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdA));
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdB));
    EXPECT_TRUE(HasIntersectionId(dut, kIntersectionIdC));
  }
  {
    const std::vector<maliput::api::Intersection*> dut(
        intersection_book_.FindIntersections({LaneSRange{api::LaneId{"lane_c_1"}, api::SRange{0., 10.}}}, kTolerance));
    EXPECT_EQ(static_cast<int>(dut.size()), 1);
    EXPECT_EQ(dut[0]->id(), kIntersectionIdC);
  }
}

TEST_F(IntersectionBookTest, FindIntersectionByTrafficLightId) {
  EXPECT_EQ(intersection_book_.FindIntersection(kTrafficLightIdAPhase1)->id(), kIntersectionIdA);
  EXPECT_EQ(intersection_book_.FindIntersection(kTrafficLightIdBPhase2)->id(), kIntersectionIdB);
  EXPECT_EQ(intersection_book_.FindIntersection(kTrafficLightIdAPhase3)->id(), kIntersectionIdC);
}

TEST_F(IntersectionBookTest, FindIntersectionByDiscreteValueRuleId) {
  EXPECT_EQ(intersection_book_.FindIntersection(kDiscreteValueRuleIdAPhase1)->id(), Intersection::Id(kIntersectionIdA));
  EXPECT_EQ(intersection_book_.FindIntersection(kDiscreteValueRuleIdBPhase2)->id(), Intersection::Id(kIntersectionIdB));
  EXPECT_EQ(intersection_book_.FindIntersection(kDiscreteValueRuleIdAPhase3)->id(), Intersection::Id(kIntersectionIdC));
}

TEST_F(IntersectionBookTest, FindIntersectionByRightOfWayRuleId) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  EXPECT_EQ(intersection_book_.FindIntersection(kRightOfWayRuleIdAPhase1)->id(), Intersection::Id(kIntersectionIdA));
  EXPECT_EQ(intersection_book_.FindIntersection(kRightOfWayRuleIdBPhase2)->id(), Intersection::Id(kIntersectionIdB));
  EXPECT_EQ(intersection_book_.FindIntersection(kRightOfWayRuleIdAPhase3)->id(), Intersection::Id(kIntersectionIdC));
#pragma GCC diagnostic pop
}

}  // namespace
}  // namespace maliput
