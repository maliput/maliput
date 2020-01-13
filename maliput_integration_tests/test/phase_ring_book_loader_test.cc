#include "maliput/base/phase_ring_book_loader.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <sstream>
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

#include "drake/common/find_resource.h"

namespace maliput {
namespace test {
namespace {

using api::RoadGeometry;
using api::rules::Bulb;
using api::rules::BulbGroup;
using api::rules::BulbState;
using api::rules::DiscreteValueRule;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
using api::rules::Rule;
using api::rules::TrafficLight;
using api::rules::TrafficLightBook;
using api::rules::UniqueBulbGroupId;
using api::rules::UniqueBulbId;

constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

// Returns the RelatedVehicleInZoneStopBehavior api::rules::Rule::RelatedRules.
//
// `rule_id_str` is the RelatedVehicleInZoneStopBehavior api::Rule::Id that the
// rule refers to.
std::pair<std::string, std::vector<Rule::Id>> RelatedVehicleInZoneStopBehavior(const std::string& rule_id_str) {
  return std::make_pair(VehicleStopInZoneBehaviorRuleTypeId().string(),
                        std::vector<Rule::Id>{Rule::Id(VehicleStopInZoneBehaviorRuleTypeId().string() + rule_id_str)});
}

// Returns the yield group of a Right-Of-Way Rule Type.
//
// `rule_id_strs` is a vector of strings containing Right-Of-Way api::Rule::Ids a
// rule yields to.
std::pair<std::string, std::vector<Rule::Id>> RelatedYieldGroup(const std::vector<std::string>& rule_id_strs) {
  std::vector<Rule::Id> rule_ids;
  for (const std::string& rule_id_str : rule_id_strs) {
    rule_ids.push_back(Rule::Id(RightOfWayRuleTypeId().string() + rule_id_str));
  }
  return std::make_pair(RelatedRulesKeys::kYieldGroup, rule_ids);
}

// Returns a api::UniqueBulbGroupId from `unique_bulb_group_id_str`.
//
// `unique_bulb_group_id_str` should have the form 'TrafficLightId-BulbGroupId'.
UniqueBulbGroupId FromString(const std::string& unique_bulb_group_id_str) {
  std::vector<std::string> ids;
  std::string id;
  std::istringstream ss(unique_bulb_group_id_str);
  while (std::getline(ss, id, *(UniqueBulbGroupId::delimiter().c_str()))) {
    ids.push_back(id);
  }
  MALIPUT_THROW_UNLESS(ids.size() == 2);
  return UniqueBulbGroupId(TrafficLight::Id(ids[0]), BulbGroup::Id(ids[1]));
}

// Returns Rule::RelatedUniqueIds initialized with `unique_bulb_group_id_str` as
// sole api::UniqueBulbGroupId.
Rule::RelatedUniqueIds CreateRelatedBulbGroups(const std::string& unique_bulb_group_id_str) {
  return Rule::RelatedUniqueIds{{RelatedUniqueIdsKeys::kBulbGroup, {FromString(unique_bulb_group_id_str)}}};
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
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthStraight"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("SouthFacing-SouthFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthStraight"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthStraight"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("NorthFacing-NorthFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastStraight"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastStraight"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("WestFacing-WestFacingBulbs"), "Stop"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestStraight"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestStraight"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("EastFacing-EastFacingBulbs"), "Stop"}},

                  {Rule::Id(RightOfWayRuleTypeId().string() + "/NorthRightTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthRightTurn"),
                                          RelatedYieldGroup({"/SouthLeftTurn"})},
                       CreateRelatedBulbGroups("SouthFacing-SouthFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthRightTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthRightTurn"),
                                          RelatedYieldGroup({"/NorthLeftTurn"})},
                       CreateRelatedBulbGroups("NorthFacing-NorthFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastRightTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastRightTurn"),
                                          RelatedYieldGroup({"/SouthStraight", "/WestLeftTurn"})},
                       CreateRelatedBulbGroups("WestFacing-WestFacingBulbs"), "StopThenGo"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestRightTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestRightTurn"),
                                          RelatedYieldGroup({"/NorthStraight", "/EastLeftTurn"})},
                       CreateRelatedBulbGroups("EastFacing-EastFacingBulbs"), "StopThenGo"}},

                  {Rule::Id(RightOfWayRuleTypeId().string() + "/NorthLeftTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthLeftTurn"),
                                          RelatedYieldGroup({"/SouthStraight"})},
                       CreateRelatedBulbGroups("SouthFacing-SouthFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthLeftTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthLeftTurn"),
                                          RelatedYieldGroup({"/NorthStraight"})},
                       CreateRelatedBulbGroups("NorthFacing-NorthFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastLeftTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastLeftTurn"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("WestFacing-WestFacingBulbs"), "Stop"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestLeftTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestLeftTurn"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("EastFacing-EastFacingBulbs"), "Stop"}}},
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
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthStraight"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("SouthFacing-SouthFacingBulbs"), "Stop"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthStraight"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthStraight"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("NorthFacing-NorthFacingBulbs"), "Stop"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastStraight"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastStraight"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("WestFacing-WestFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestStraight"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestStraight"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("EastFacing-EastFacingBulbs"), "Go"}},

                  {Rule::Id(RightOfWayRuleTypeId().string() + "/NorthRightTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthRightTurn"),
                                          RelatedYieldGroup({"/EastStraight", "/SouthLeftTurn"})},
                       CreateRelatedBulbGroups("SouthFacing-SouthFacingBulbs"), "StopThenGo"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthRightTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthRightTurn"),
                                          RelatedYieldGroup({"/WestStraight", "/NorthLeftTurn"})},
                       CreateRelatedBulbGroups("NorthFacing-NorthFacingBulbs"), "StopThenGo"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastRightTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastRightTurn"),
                                          RelatedYieldGroup({"/WestLeftTurn"})},
                       CreateRelatedBulbGroups("WestFacing-WestFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestRightTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestRightTurn"),
                                          RelatedYieldGroup({"/EastLeftTurn"})},
                       CreateRelatedBulbGroups("EastFacing-EastFacingBulbs"), "Go"}},

                  {Rule::Id(RightOfWayRuleTypeId().string() + "/NorthLeftTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/NorthLeftTurn"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("SouthFacing-SouthFacingBulbs"), "Stop"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/SouthLeftTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/SouthLeftTurn"), RelatedYieldGroup({})},
                       CreateRelatedBulbGroups("NorthFacing-NorthFacingBulbs"), "Stop"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/EastLeftTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/EastLeftTurn"),
                                          RelatedYieldGroup({"/WestStraight"})},
                       CreateRelatedBulbGroups("WestFacing-WestFacingBulbs"), "Go"}},
                  {Rule::Id(RightOfWayRuleTypeId().string() + "/WestLeftTurn"),
                   DiscreteValueRule::DiscreteValue{
                       Rule::State::kStrict,
                       Rule::RelatedRules{RelatedVehicleInZoneStopBehavior("/WestLeftTurn"),
                                          RelatedYieldGroup({"/EastStraight"})},
                       CreateRelatedBulbGroups("EastFacing-EastFacingBulbs"), "Go"}}},
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
                               {Phase::Id("EastWestPhase"), {{Phase::Id("NorthSouthPhase"), std::nullopt}}}}) {}

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
  const std::optional<PhaseRing> ring = phase_ring_book->GetPhaseRing(PhaseRing::Id(ring_id));
  EXPECT_NE(ring, std::nullopt);
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
