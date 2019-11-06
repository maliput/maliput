#include "maliput/base/phase_ring_book_loader.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/road_rulebook_loader.h"
#include "maliput/base/rule_registry.h"
#include "maliput/base/traffic_light_book_loader.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/phases_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "multilane/builder.h"
#include "multilane/loader.h"

#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"

namespace maliput {
namespace test {
namespace {

using api::RoadGeometry;
using api::rules::Bulb;
using api::rules::BulbGroup;
using api::rules::BulbState;
using api::rules::MakeDiscreteValue;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
using api::rules::Rule;
using api::rules::TrafficLight;
using api::rules::TrafficLightBook;
using api::rules::UniqueBulbId;

constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

std::pair<std::string, std::vector<Rule::Id>> RelatedVehicleInZoneStopBehavior(const std::string& row_rule_name) {
  return std::make_pair(
      VehicleStopInZoneBehaviorRuleTypeId().string(),
      std::vector<Rule::Id>{Rule::Id(VehicleStopInZoneBehaviorRuleTypeId().string() + row_rule_name)});
}

std::pair<std::string, std::vector<Rule::Id>> RelatedYieldGroup(const std::vector<std::string>& row_rule_names) {
  std::vector<Rule::Id> rule_ids;
  for (const std::string& row_rule_name : row_rule_names) {
    rule_ids.push_back(Rule::Id(RightOfWayRuleTypeId().string() + row_rule_name));
  }
  return std::make_pair(RightOfWayYieldGroup(), rule_ids);
}

class TestLoading2x2IntersectionPhasebook : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionPhasebook()
      : filepath_(maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_VAR) + "/2x2_intersection.yaml"),
        road_geometry_(multilane::LoadFile(multilane::BuilderFactory(), filepath_)),
        rulebook_(LoadRoadRulebookFromFile(road_geometry_.get(), filepath_)),
        traffic_light_book_(LoadTrafficLightBookFromFile(filepath_)),
        expected_phases_(
            {Phase(
                 Phase::Id("NorthSouthPhase"),
                 {{RightOfWayRule::Id("NorthStraight"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("SouthStraight"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("EastStraight"), RightOfWayRule::State::Id("Stop")},
                  {RightOfWayRule::Id("WestStraight"), RightOfWayRule::State::Id("Stop")},
                  {RightOfWayRule::Id("NorthRightTurn"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("SouthRightTurn"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("EastRightTurn"), RightOfWayRule::State::Id("StopThenGo")},
                  {RightOfWayRule::Id("WestRightTurn"), RightOfWayRule::State::Id("StopThenGo")},
                  {RightOfWayRule::Id("NorthLeftTurn"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("SouthLeftTurn"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("EastLeftTurn"), RightOfWayRule::State::Id("Stop")},
                  {RightOfWayRule::Id("WestLeftTurn"), RightOfWayRule::State::Id("Stop")}},
                 {{Rule::Id(RightOfWayRuleTypeId().string() + "/NorthStraight"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthStraight"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthStraight"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthStraight"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastStraight"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastStraight"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Stop")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestStraight"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestStraight"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Stop")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/NorthRightTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthRightTurn"),
                                                        RelatedYieldGroup({"/SouthLeftTurn"})},
                                     Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthRightTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthRightTurn"),
                                                        RelatedYieldGroup({"/NorthLeftTurn"})},
                                     Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastRightTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastRightTurn"),
                                                        RelatedYieldGroup({"/SouthStraight", "/WestLeftTurn"})},
                                     Rule::RelatedUniqueIds{}, "StopThenGo")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestRightTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestRightTurn"),
                                                        RelatedYieldGroup({"/NorthStraight", "/EastLeftTurn"})},
                                     Rule::RelatedUniqueIds{}, "StopThenGo")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/NorthLeftTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthLeftTurn"),
                                                        RelatedYieldGroup({"/SouthStraight"})},
                                     Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthLeftTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthLeftTurn"),
                                                        RelatedYieldGroup({"/NorthStraight"})},
                                     Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastLeftTurn"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastLeftTurn"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Stop")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestLeftTurn"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestLeftTurn"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Stop")}},
                 {{{UniqueBulbId{TrafficLight::Id("EastFacing"), BulbGroup::Id("EastFacingBulbs"), Bulb::Id("RedBulb")},
                    BulbState::kOn},
                   {UniqueBulbId{TrafficLight::Id("EastFacing"), BulbGroup::Id("EastFacingBulbs"),
                                 Bulb::Id("YellowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("EastFacing"), BulbGroup::Id("EastFacingBulbs"),
                                 Bulb::Id("GreenBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("EastFacing"), BulbGroup::Id("EastFacingBulbs"),
                                 Bulb::Id("YellowLeftArrowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("NorthFacing"), BulbGroup::Id("NorthFacingBulbs"),
                                 Bulb::Id("RedBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("NorthFacing"), BulbGroup::Id("NorthFacingBulbs"),
                                 Bulb::Id("YellowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("NorthFacing"), BulbGroup::Id("NorthFacingBulbs"),
                                 Bulb::Id("GreenBulb")},
                    BulbState::kOn},
                   {UniqueBulbId{TrafficLight::Id("NorthFacing"), BulbGroup::Id("NorthFacingBulbs"),
                                 Bulb::Id("YellowLeftArrowBulb")},
                    BulbState::kBlinking},
                   {UniqueBulbId{TrafficLight::Id("SouthFacing"), BulbGroup::Id("SouthFacingBulbs"),
                                 Bulb::Id("RedBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("SouthFacing"), BulbGroup::Id("SouthFacingBulbs"),
                                 Bulb::Id("YellowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("SouthFacing"), BulbGroup::Id("SouthFacingBulbs"),
                                 Bulb::Id("GreenBulb")},
                    BulbState::kOn},
                   {UniqueBulbId{TrafficLight::Id("SouthFacing"), BulbGroup::Id("SouthFacingBulbs"),
                                 Bulb::Id("YellowLeftArrowBulb")},
                    BulbState::kBlinking},
                   {UniqueBulbId{TrafficLight::Id("WestFacing"), BulbGroup::Id("WestFacingBulbs"), Bulb::Id("RedBulb")},
                    BulbState::kOn},
                   {UniqueBulbId{TrafficLight::Id("WestFacing"), BulbGroup::Id("WestFacingBulbs"),
                                 Bulb::Id("YellowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("WestFacing"), BulbGroup::Id("WestFacingBulbs"),
                                 Bulb::Id("GreenBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("WestFacing"), BulbGroup::Id("WestFacingBulbs"),
                                 Bulb::Id("YellowLeftArrowBulb")},
                    BulbState::kOff}}}),
             Phase(
                 Phase::Id("EastWestPhase"),
                 {{RightOfWayRule::Id("NorthStraight"), RightOfWayRule::State::Id("Stop")},
                  {RightOfWayRule::Id("SouthStraight"), RightOfWayRule::State::Id("Stop")},
                  {RightOfWayRule::Id("EastStraight"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("WestStraight"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("NorthRightTurn"), RightOfWayRule::State::Id("StopThenGo")},
                  {RightOfWayRule::Id("SouthRightTurn"), RightOfWayRule::State::Id("StopThenGo")},
                  {RightOfWayRule::Id("EastRightTurn"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("WestRightTurn"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("NorthLeftTurn"), RightOfWayRule::State::Id("Stop")},
                  {RightOfWayRule::Id("SouthLeftTurn"), RightOfWayRule::State::Id("Stop")},
                  {RightOfWayRule::Id("EastLeftTurn"), RightOfWayRule::State::Id("Go")},
                  {RightOfWayRule::Id("WestLeftTurn"), RightOfWayRule::State::Id("Go")}},
                 {{Rule::Id(RightOfWayRuleTypeId().string() + "/NorthStraight"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthStraight"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Stop")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthStraight"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthStraight"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Stop")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastStraight"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastStraight"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestStraight"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestStraight"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/NorthRightTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthRightTurn"),
                                                        RelatedYieldGroup({"/EastStraight", "/SouthLeftTurn"})},
                                     Rule::RelatedUniqueIds{}, "StopThenGo")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthRightTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthRightTurn"),
                                                        RelatedYieldGroup({"/WestStraight", "/NorthLeftTurn"})},
                                     Rule::RelatedUniqueIds{}, "StopThenGo")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastRightTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastRightTurn"),
                                                        RelatedYieldGroup({"/WestLeftTurn"})},
                                     Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestRightTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestRightTurn"),
                                                        RelatedYieldGroup({"/EastLeftTurn"})},
                                     Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/NorthLeftTurn"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthLeftTurn"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Stop")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthLeftTurn"),
                   MakeDiscreteValue(
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthLeftTurn"), RelatedYieldGroup({})},
                       Rule::RelatedUniqueIds{}, "Stop")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastLeftTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastLeftTurn"),
                                                        RelatedYieldGroup({"/WestStraight"})},
                                     Rule::RelatedUniqueIds{}, "Go")},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestLeftTurn"),
                   MakeDiscreteValue(Rule::State::kStrict,
                                     Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestLeftTurn"),
                                                        RelatedYieldGroup({"/EastStraight"})},
                                     Rule::RelatedUniqueIds{}, "Go")}},
                 {{{UniqueBulbId{TrafficLight::Id("WestFacing"), BulbGroup::Id("WestFacingBulbs"), Bulb::Id("RedBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("WestFacing"), BulbGroup::Id("WestFacingBulbs"),
                                 Bulb::Id("YellowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("WestFacing"), BulbGroup::Id("WestFacingBulbs"),
                                 Bulb::Id("GreenBulb")},
                    BulbState::kOn},
                   {UniqueBulbId{TrafficLight::Id("WestFacing"), BulbGroup::Id("WestFacingBulbs"),
                                 Bulb::Id("YellowLeftArrowBulb")},
                    BulbState::kBlinking},
                   {UniqueBulbId{TrafficLight::Id("EastFacing"), BulbGroup::Id("EastFacingBulbs"), Bulb::Id("RedBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("EastFacing"), BulbGroup::Id("EastFacingBulbs"),
                                 Bulb::Id("YellowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("EastFacing"), BulbGroup::Id("EastFacingBulbs"),
                                 Bulb::Id("GreenBulb")},
                    BulbState::kOn},
                   {UniqueBulbId{TrafficLight::Id("EastFacing"), BulbGroup::Id("EastFacingBulbs"),
                                 Bulb::Id("YellowLeftArrowBulb")},
                    BulbState::kBlinking},
                   {UniqueBulbId{TrafficLight::Id("NorthFacing"), BulbGroup::Id("NorthFacingBulbs"),
                                 Bulb::Id("RedBulb")},
                    BulbState::kOn},
                   {UniqueBulbId{TrafficLight::Id("NorthFacing"), BulbGroup::Id("NorthFacingBulbs"),
                                 Bulb::Id("YellowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("NorthFacing"), BulbGroup::Id("NorthFacingBulbs"),
                                 Bulb::Id("GreenBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("NorthFacing"), BulbGroup::Id("NorthFacingBulbs"),
                                 Bulb::Id("YellowLeftArrowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("SouthFacing"), BulbGroup::Id("SouthFacingBulbs"),
                                 Bulb::Id("RedBulb")},
                    BulbState::kOn},
                   {UniqueBulbId{TrafficLight::Id("SouthFacing"), BulbGroup::Id("SouthFacingBulbs"),
                                 Bulb::Id("YellowBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("SouthFacing"), BulbGroup::Id("SouthFacingBulbs"),
                                 Bulb::Id("GreenBulb")},
                    BulbState::kOff},
                   {UniqueBulbId{TrafficLight::Id("SouthFacing"), BulbGroup::Id("SouthFacingBulbs"),
                                 Bulb::Id("YellowLeftArrowBulb")},
                    BulbState::kOff}}})}),
        expected_next_phases_({{Phase::Id("NorthSouthPhase"), {{Phase::Id("EastWestPhase"), 45.0}}},
                               {Phase::Id("EastWestPhase"), {{Phase::Id("NorthSouthPhase"), drake::nullopt}}}}) {}

  const std::string filepath_;
  const std::unique_ptr<const RoadGeometry> road_geometry_;
  const std::unique_ptr<const RoadRulebook> rulebook_;
  const std::unique_ptr<const TrafficLightBook> traffic_light_book_;
  const std::vector<Phase> expected_phases_;
  const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>> expected_next_phases_;
};

TEST_F(TestLoading2x2IntersectionPhasebook, LoadFromFile) {
  std::unique_ptr<PhaseRingBook> phase_ring_book =
      LoadPhaseRingBookFromFile(rulebook_.get(), traffic_light_book_.get(), filepath_);
  EXPECT_NE(phase_ring_book, nullptr);
  const PhaseRing::Id ring_id("2x2Intersection");
  const drake::optional<PhaseRing> ring = phase_ring_book->GetPhaseRing(PhaseRing::Id(ring_id));
  EXPECT_NE(ring, drake::nullopt);
  EXPECT_EQ(ring->id(), ring_id);
  const auto& phases = ring->phases();
  EXPECT_EQ(phases.size(), expected_phases_.size());

  for (const auto& expected_phase : expected_phases_) {
    EXPECT_TRUE(MALIPUT_IS_EQUAL(expected_phase, phases.at(expected_phase.id())));
  }

  const std::unordered_map<Phase::Id, std::vector<PhaseRing::NextPhase>>& next_phases = ring->next_phases();
  EXPECT_EQ(next_phases.size(), expected_next_phases_.size());
  for (const auto& expected_next_phase : expected_next_phases_) {
    EXPECT_TRUE(MALIPUT_IS_EQUAL(expected_next_phase.second, next_phases.at(expected_next_phase.first)));
  }
}

}  // namespace
}  // namespace test
}  // namespace maliput
