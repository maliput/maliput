#include "maliput/base/phase_ring_book_loader.h"

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/road_rulebook_loader.h"
#include "maliput/base/traffic_light_book_loader.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/phases_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "multilane/builder.h"
#include "multilane/loader.h"

#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"

namespace maliput {
namespace {

using api::RoadGeometry;
using api::rules::Bulb;
using api::rules::BulbGroup;
using api::rules::BulbState;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

using api::rules::TrafficLight;
using api::rules::TrafficLightBook;
using api::rules::UniqueBulbId;

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
  EXPECT_EQ(phases.size(), 2);

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
}  // namespace maliput
