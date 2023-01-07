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
#include "maliput/base/phase_ring_book_loader.h"

#include <fstream>
#include <memory>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/api/unique_id.h"
#include "maliput/base/manual_rulebook.h"
#include "maliput/base/rule_registry.h"
#include "maliput/base/traffic_light_book.h"
#include "maliput/common/filesystem.h"

namespace maliput {
namespace {

using maliput::api::InertialPosition;
using maliput::api::LaneSRoute;
using maliput::api::Rotation;
using maliput::api::UniqueId;
using maliput::api::rules::Bulb;
using maliput::api::rules::BulbColor;
using maliput::api::rules::BulbGroup;
using maliput::api::rules::BulbState;
using maliput::api::rules::BulbStates;
using maliput::api::rules::BulbType;
using maliput::api::rules::DiscreteValueRule;
using maliput::api::rules::DiscreteValueRuleStates;
using maliput::api::rules::Phase;
using maliput::api::rules::PhaseRing;
using maliput::api::rules::Rule;
using maliput::api::rules::TrafficLight;

// Helpers methods for describing a RoadRulebook and a TrafficLightBook
// for a sample RoadNetwork:
//                              \\\/()=> TrafficLight Facing East
//  Lane_0_0_2 ----<------<-----\\\-----<-----<---- Lane_0_1_2
//             -----------------\\\----------------
//  Lane_0_0_1 ---->------>-----\\\----->----->---- Lane_0_1_1
//                          <=()\\\/  TrafficLight Facing West
//
// 2-way road that is intersected by a pedestrian zebra path in the middle:
// - Four driving lanes are described:
//   - Lane_0_0_1: West-East direction, before zebra.
//   - Lane_0_1_1: West-East direction, after zebra.
//   - Lane_0_1_2: East-West direction, before zebra.
//   - Lane_0_0_2: East-West direction, after zebra.
// - Two traffic lights are identified:
//   - Facing West: Affecting Lane_0_0_1 Right of way
//   - Facing East: Affecting Lane_0_1_2 Right of way
//
class StraightRoadNetworkHelpers {
 public:
  // Returns DiscreteValueRule of type maliput::RightOfWayRuleTypeId() for lane 0_0_1.
  static DiscreteValueRule GetWestEastDiscreteRule() {
    return {Rule::Id{maliput::RightOfWayRuleTypeId().string() + "/0_0_1"},
            maliput::RightOfWayRuleTypeId(),
            LaneSRoute{{{maliput::api::LaneId("0_0_1"), {0, 10.}}}},
            {{Rule::State::kBestEffort,
              {},
              {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {UniqueId("WestFacing-WestFacingBulbs")}}},
              "Go"},
             {Rule::State::kStrict,
              {},
              {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {UniqueId("WestFacing-WestFacingBulbs")}}},
              "Stop"}}};
  }

  // Returns DiscreteValueRule of type maliput::RightOfWayRuleTypeId() for lane 0_1_2.
  static DiscreteValueRule GetEastWestDiscreteRule() {
    return {Rule::Id{maliput::RightOfWayRuleTypeId().string() + "/0_1_2"},
            maliput::RightOfWayRuleTypeId(),
            LaneSRoute{{{maliput::api::LaneId("0_1_2"), {0, 10.}}}},
            {{Rule::State::kBestEffort,
              {},
              {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {UniqueId("EastFacing-EastFacingBulbs")}}},
              "Go"},
             {Rule::State::kStrict,
              {},
              {{maliput::RelatedUniqueIdsKeys::kBulbGroup, {UniqueId("EastFacing-EastFacingBulbs")}}},
              "Stop"}}};
  }

  // Returns a RoadRulebook containing RightOfWay rules that apply to the proposed RoadNetwork sample.
  static std::unique_ptr<maliput::api::rules::RoadRulebook> CreateRoadRulebook() {
    std::unique_ptr<maliput::ManualRulebook> rulebook = std::make_unique<maliput::ManualRulebook>();
    rulebook->AddRule(GetEastWestDiscreteRule());
    rulebook->AddRule(GetWestEastDiscreteRule());
    return rulebook;
  }

  // Returns a TrafficLight facing West based on the proposed RoadNetwork sample.
  static std::unique_ptr<const TrafficLight> CreateWestFacingTrafficLight() {
    std::vector<std::unique_ptr<BulbGroup>> bulb_groups;
    bulb_groups.push_back(CreateBulbGroup("WestFacingBulbs"));
    return std::make_unique<const TrafficLight>(TrafficLight::Id("WestFacing"), InertialPosition{10., 0., 2.},
                                                Rotation::FromRpy(0., 0., M_PI), std::move(bulb_groups));
  }

  // Returns a TrafficLight facing East based on the proposed RoadNetwork sample.
  static std::unique_ptr<const TrafficLight> CreateEastFacingTrafficLight() {
    std::vector<std::unique_ptr<BulbGroup>> bulb_groups;
    bulb_groups.push_back(CreateBulbGroup("EastFacingBulbs"));
    return std::make_unique<const TrafficLight>(TrafficLight::Id("EastFacing"), InertialPosition{10., 4., 2.},
                                                Rotation::FromRpy(0., 0., 0.), std::move(bulb_groups));
  }

  // Returns a TrafficLightBook based on the proposed RoadNetwork sample.
  static std::unique_ptr<maliput::api::rules::TrafficLightBook> CreateTrafficLightBook() {
    std::unique_ptr<maliput::TrafficLightBook> traffic_light_book = std::make_unique<maliput::TrafficLightBook>();
    // Facing West
    traffic_light_book->AddTrafficLight(CreateWestFacingTrafficLight());
    // Facing East
    traffic_light_book->AddTrafficLight(CreateEastFacingTrafficLight());
    return traffic_light_book;
  }

  // Returns a YAML description based on the proposed RoadNetwork sample.
  static std::string PhaseRingBookYamlDescription() {
    return fmt::format(
        R"R(PhaseRings:
- ID: StraightRoadCrossingPath
  Rules: [{0}, {1}]
  Phases:
  - ID: AllGo
    RightOfWayRuleStates: {{{0}: Go, {1}: Go}}
    TrafficLightStates:
      WestFacing:
          WestFacingBulbs: {{RedBulb: Off, YellowBulb: Off, GreenBulb: On}}
      EastFacing:
          EastFacingBulbs: {{RedBulb: Off, YellowBulb: Off, GreenBulb: On}}
  - ID: AllStop
    RightOfWayRuleStates: {{{0}: Stop, {1}: Stop}}
    TrafficLightStates:
      WestFacing:
          WestFacingBulbs: {{RedBulb: On, YellowBulb: Off, GreenBulb: Off}}
      EastFacing:
          EastFacingBulbs: {{RedBulb: On, YellowBulb: Off, GreenBulb: Off}}
  PhaseTransitionGraph:
    AllGo:
    - ID: AllStop
      duration_until: 45
    AllStop:
    - ID: AllGo
)R",
        maliput::RightOfWayRuleTypeId().string() + "/0_0_1", maliput::RightOfWayRuleTypeId().string() + "/0_1_2");
  }

  // Returns a YAML description based on the proposed RoadNetwork sample without RightOfWayRuleStates.
  static std::string PhaseRingBookYamlDescriptionWithoutRightOfWayRuleStates() {
    return fmt::format(
        R"R(PhaseRings:
- ID: StraightRoadCrossingPath
  Rules: [{0}, {1}]
  Phases:
  - ID: AllGo
    RightOfWayRuleStates: {2}
    TrafficLightStates:
      WestFacing:
          WestFacingBulbs: {{RedBulb: Off, YellowBulb: Off, GreenBulb: On}}
      EastFacing:
          EastFacingBulbs: {{RedBulb: Off, YellowBulb: Off, GreenBulb: On}}
  - ID: AllStop
    TrafficLightStates:
      WestFacing:
          WestFacingBulbs: {{RedBulb: On, YellowBulb: Off, GreenBulb: Off}}
      EastFacing:
          EastFacingBulbs: {{RedBulb: On, YellowBulb: Off, GreenBulb: Off}}
  PhaseTransitionGraph:
    AllGo:
    - ID: AllStop
      duration_until: 45
    AllStop:
    - ID: AllGo
)R",
        maliput::RightOfWayRuleTypeId().string() + "/0_0_1", maliput::RightOfWayRuleTypeId().string() + "/0_1_2",
        "{}" /* Empty map */);
  }

 private:
  // Creates a 3-Round-Bulb BulbGroup of id `bulb_group_id`.
  static std::unique_ptr<BulbGroup> CreateBulbGroup(const std::string bulb_group_id) {
    std::vector<std::unique_ptr<Bulb>> bulbs;
    bulbs.push_back(std::make_unique<Bulb>(Bulb::Id{"GreenBulb"}, InertialPosition{0., 0., -0.25},
                                           Rotation::FromRpy(0., 0., 0.), BulbColor::kGreen, BulbType::kRound));
    bulbs.push_back(std::make_unique<Bulb>(Bulb::Id{"YellowBulb"}, InertialPosition{0., 0., 0.},
                                           Rotation::FromRpy(0., 0., 0.), BulbColor::kYellow, BulbType::kRound));
    bulbs.push_back(std::make_unique<Bulb>(Bulb::Id{"RedBulb"}, InertialPosition{0., 0., 0.25},
                                           Rotation::FromRpy(0., 0., 0.), BulbColor::kRed, BulbType::kRound));
    return std::make_unique<BulbGroup>(BulbGroup::Id{bulb_group_id},
                                       InertialPosition{
                                           0.,
                                           0.,
                                           0.,
                                       },
                                       Rotation::FromRpy(0., 0., 0.), std::move(bulbs));
  }
};

// Evaluates the phase ring book loader
//
// A temporary file is created containing the PhaseRingBook YAML description.
// The test will remove the temporary
// file when PhaseRingBookLoaderFromFileTest::TearDown() is called.
//
// The PhaseRingBook needs RoadRulebook and TrafficLightBook entities properly set:
//  - RoadRulebook: See StraightRoadNetworkHelpers::CreateRoadRulebook()
//  - TrafficLightBook: See StraightRoadNetworkHelpers::CreateTrafficLightBook()
// The PhaseRingBook is populated using a YAML description:
//  - YAML description: See See StraightRoadNetworkHelpers::PhaseRingBookYamlDescription()
class PhaseRingBookLoaderFromFileTest : public ::testing::Test {
 public:
  static void GenerateYamlFileFromString(const std::string& string_to_yaml, const std::string& filepath) {
    std::ofstream os(filepath);
    os << string_to_yaml;
  }

  void SetUp() override {
    directory_.set_as_temp();
    directory_.append("PhaseRingBookLoaderTest");
    ASSERT_TRUE(common::Filesystem::create_directory(directory_));

    phase_ring_book_string_ = StraightRoadNetworkHelpers::PhaseRingBookYamlDescription();
    filepath_ = directory_.get_path() + "/phase_ring_book_test.yaml";
    GenerateYamlFileFromString(phase_ring_book_string_, filepath_);
  }

  void TearDown() override {
    if (!filepath_.empty()) {
      EXPECT_TRUE(common::Filesystem::remove_file(common::Path(filepath_)));
    }
    ASSERT_TRUE(common::Filesystem::remove_directory(directory_));
  }

  const DiscreteValueRule kEastWestDiscreteRule{StraightRoadNetworkHelpers::GetEastWestDiscreteRule()};
  const DiscreteValueRule kWestEastDiscreteRule{StraightRoadNetworkHelpers::GetWestEastDiscreteRule()};
  const std::unique_ptr<const TrafficLight> kWestFacingTrafficLight{
      StraightRoadNetworkHelpers::CreateWestFacingTrafficLight()};
  const std::unique_ptr<const TrafficLight> kEastFacingTrafficLight{
      StraightRoadNetworkHelpers::CreateEastFacingTrafficLight()};

  const DiscreteValueRuleStates expected_discrete_value_rule_states_all_go_phase{
      {kEastWestDiscreteRule.id(), kEastWestDiscreteRule.states()[0]},
      {kWestEastDiscreteRule.id(), kWestEastDiscreteRule.states()[0]}};

  const BulbStates expected_bulb_states_all_go_phase{
      // West facing bulbs.
      {kWestFacingTrafficLight->bulb_groups()[0]->bulbs()[0]->unique_id(), BulbState::kOn},   // Green
      {kWestFacingTrafficLight->bulb_groups()[0]->bulbs()[1]->unique_id(), BulbState::kOff},  // Yellow
      {kWestFacingTrafficLight->bulb_groups()[0]->bulbs()[2]->unique_id(), BulbState::kOff},  // Red
      // East facing bulbs.
      {kEastFacingTrafficLight->bulb_groups()[0]->bulbs()[0]->unique_id(), BulbState::kOn},   // Green
      {kEastFacingTrafficLight->bulb_groups()[0]->bulbs()[1]->unique_id(), BulbState::kOff},  // Yellow
      {kEastFacingTrafficLight->bulb_groups()[0]->bulbs()[2]->unique_id(), BulbState::kOff},  // Red
  };

  const DiscreteValueRuleStates expected_discrete_value_rule_states_all_stop_phase{
      {kEastWestDiscreteRule.id(), kEastWestDiscreteRule.states()[1]},
      {kWestEastDiscreteRule.id(), kWestEastDiscreteRule.states()[1]}};

  const BulbStates expected_bulb_states_all_stop_phase{
      // West facing bulbs.
      {kWestFacingTrafficLight->bulb_groups()[0]->bulbs()[0]->unique_id(), BulbState::kOff},  // Green
      {kWestFacingTrafficLight->bulb_groups()[0]->bulbs()[1]->unique_id(), BulbState::kOff},  // Yellow
      {kWestFacingTrafficLight->bulb_groups()[0]->bulbs()[2]->unique_id(), BulbState::kOn},   // Red
      // East facing bulbs.
      {kEastFacingTrafficLight->bulb_groups()[0]->bulbs()[0]->unique_id(), BulbState::kOff},  // Green
      {kEastFacingTrafficLight->bulb_groups()[0]->bulbs()[1]->unique_id(), BulbState::kOff},  // Yellow
      {kEastFacingTrafficLight->bulb_groups()[0]->bulbs()[2]->unique_id(), BulbState::kOn},   // Red
  };

  common::Path directory_;
  std::string phase_ring_book_string_{};
  std::string filepath_{};
};

// Uses maliput::LoadPhaseRingBookFromFile() to populate the PhaseRingBook via yaml file description.
TEST_F(PhaseRingBookLoaderFromFileTest, PhaseRingBookTest) {
  const auto rulebook = StraightRoadNetworkHelpers::CreateRoadRulebook();
  const auto traffic_light_book = StraightRoadNetworkHelpers::CreateTrafficLightBook();
  std::unique_ptr<maliput::api::rules::PhaseRingBook> dut;

  ASSERT_NO_THROW(dut = maliput::LoadPhaseRingBookFromFile(rulebook.get(), traffic_light_book.get(), filepath_));

  // PhaseRingBook should contain only one PhaseRing.
  EXPECT_EQ(1, static_cast<int>(dut->GetPhaseRings().size()));
  // Verifies that Rules Ids are located within a PhaseRing.
  EXPECT_NE(std::nullopt, dut->FindPhaseRing(StraightRoadNetworkHelpers::GetEastWestDiscreteRule().id()));
  EXPECT_NE(std::nullopt, dut->FindPhaseRing(StraightRoadNetworkHelpers::GetWestEastDiscreteRule().id()));

  const PhaseRing::Id kPhaseRingId{"StraightRoadCrossingPath"};
  const Phase::Id kAllGoPhaseId{"AllGo"};
  const Phase::Id kAllStopPhaseId{"AllStop"};

  const std::optional<PhaseRing> phase_ring = dut->GetPhaseRing(dut->GetPhaseRings()[0]);
  ASSERT_EQ(kPhaseRingId, phase_ring->id());

  // Verifies phases.
  const auto all_go_phase{phase_ring->GetPhase(kAllGoPhaseId)};
  ASSERT_NE(std::nullopt, all_go_phase);
  EXPECT_EQ(expected_discrete_value_rule_states_all_go_phase, all_go_phase->discrete_value_rule_states());
  ASSERT_NE(std::nullopt, all_go_phase->bulb_states());
  EXPECT_EQ(expected_bulb_states_all_go_phase, all_go_phase->bulb_states().value());

  const auto all_stop_phase{phase_ring->GetPhase(kAllStopPhaseId)};
  ASSERT_NE(std::nullopt, all_stop_phase);
  EXPECT_EQ(expected_discrete_value_rule_states_all_stop_phase, all_stop_phase->discrete_value_rule_states());
  ASSERT_NE(std::nullopt, all_stop_phase->bulb_states());
  EXPECT_EQ(expected_bulb_states_all_stop_phase, all_stop_phase->bulb_states());

  // Verifies next phases.
  const auto next_phases_from_all_go{phase_ring->GetNextPhases(kAllGoPhaseId)};
  ASSERT_EQ(1, static_cast<int>(next_phases_from_all_go.size()));
  EXPECT_EQ(kAllStopPhaseId, next_phases_from_all_go[0].id);
  ASSERT_NE(std::nullopt, next_phases_from_all_go[0].duration_until);
  EXPECT_DOUBLE_EQ(45., next_phases_from_all_go[0].duration_until.value());

  const auto next_phases_from_all_stop{phase_ring->GetNextPhases(kAllStopPhaseId)};
  ASSERT_EQ(1, static_cast<int>(next_phases_from_all_stop.size()));
  EXPECT_EQ(kAllGoPhaseId, next_phases_from_all_stop[0].id);
  EXPECT_EQ(std::nullopt, next_phases_from_all_stop[0].duration_until);
}

// PhaseRingBook contains information for both rules and traffic lights.
// When no rules are provided, the PhaseRingBook still have information about the phases that can be used.
class PhaseRingBookTestWithNoRuleBook : public PhaseRingBookLoaderFromFileTest {
 public:
  void SetUp() override {
    directory_.set_as_temp();
    directory_.append("PhaseRingBookLoaderTest");
    ASSERT_TRUE(common::Filesystem::create_directory(directory_));

    phase_ring_book_string_ = StraightRoadNetworkHelpers::PhaseRingBookYamlDescriptionWithoutRightOfWayRuleStates();
    filepath_ = directory_.get_path() + "/phase_ring_book_test.yaml";
    GenerateYamlFileFromString(phase_ring_book_string_, filepath_);
  }

  void TearDown() override {
    if (!filepath_.empty()) {
      EXPECT_TRUE(common::Filesystem::remove_file(common::Path(filepath_)));
    }
    ASSERT_TRUE(common::Filesystem::remove_directory(directory_));
  }
};

TEST_F(PhaseRingBookTestWithNoRuleBook, PhaseRingBookTestWithNoRules) {
  const auto rulebook = std::make_unique<maliput::ManualRulebook>();
  const auto traffic_light_book = StraightRoadNetworkHelpers::CreateTrafficLightBook();
  std::unique_ptr<maliput::api::rules::PhaseRingBook> dut;
  ASSERT_NO_THROW(dut = maliput::LoadPhaseRingBookFromFile(rulebook.get(), traffic_light_book.get(), filepath_));
}

}  // namespace

}  // namespace maliput
