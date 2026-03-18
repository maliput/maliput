// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include "maliput/base/traffic_light_book_loader.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/rules/traffic_lights.h"

namespace maliput {
namespace {

using api::LaneId;
using api::rules::TrafficLight;

// Minimal YAML with one traffic light that has RelatedLanes.
constexpr const char* kYamlWithRelatedLanes = R"R(
TrafficLights:
- ID: MyTrafficLight
  Pose:
    position_road_network: [1.0, 2.0, 3.0]
    orientation_road_network: [1.0, 0.0, 0.0, 0.0]
  BulbGroups:
  - ID: MyBulbGroup
    Pose:
      position_traffic_light: [0.0, 0.0, 0.0]
      orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
    Bulbs:
    - ID: RedBulb
      Pose:
        position_bulb_group: [0.0, 0.0, 0.0]
        orientation_bulb_group: [1.0, 0.0, 0.0, 0.0]
      Color: Red
      Type: Round
  RelatedLanes:
  - lane_north
  - lane_south
)R";

// Minimal YAML with one traffic light that omits RelatedLanes.
constexpr const char* kYamlWithoutRelatedLanes = R"R(
TrafficLights:
- ID: MyTrafficLight
  Pose:
    position_road_network: [1.0, 2.0, 3.0]
    orientation_road_network: [1.0, 0.0, 0.0, 0.0]
  BulbGroups:
  - ID: MyBulbGroup
    Pose:
      position_traffic_light: [0.0, 0.0, 0.0]
      orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
    Bulbs:
    - ID: RedBulb
      Pose:
        position_bulb_group: [0.0, 0.0, 0.0]
        orientation_bulb_group: [1.0, 0.0, 0.0, 0.0]
      Color: Red
      Type: Round
)R";

// YAML with two traffic lights having overlapping lane associations for
// testing FindByLane via the loader.
constexpr const char* kYamlTwoTrafficLights = R"R(
TrafficLights:
- ID: TrafficLightA
  Pose:
    position_road_network: [0.0, 0.0, 0.0]
    orientation_road_network: [1.0, 0.0, 0.0, 0.0]
  BulbGroups:
  - ID: BulbGroupA
    Pose:
      position_traffic_light: [0.0, 0.0, 0.0]
      orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
    Bulbs:
    - ID: RedBulb
      Pose:
        position_bulb_group: [0.0, 0.0, 0.0]
        orientation_bulb_group: [1.0, 0.0, 0.0, 0.0]
      Color: Red
      Type: Round
  RelatedLanes:
  - lane_1
  - lane_2
- ID: TrafficLightB
  Pose:
    position_road_network: [10.0, 0.0, 0.0]
    orientation_road_network: [1.0, 0.0, 0.0, 0.0]
  BulbGroups:
  - ID: BulbGroupB
    Pose:
      position_traffic_light: [0.0, 0.0, 0.0]
      orientation_traffic_light: [1.0, 0.0, 0.0, 0.0]
    Bulbs:
    - ID: GreenBulb
      Pose:
        position_bulb_group: [0.0, 0.0, 0.0]
        orientation_bulb_group: [1.0, 0.0, 0.0, 0.0]
      Color: Green
      Type: Round
  RelatedLanes:
  - lane_2
  - lane_3
)R";

GTEST_TEST(TrafficLightBookLoaderTest, LoadWithRelatedLanes) {
  const auto dut = LoadTrafficLightBook(std::string(kYamlWithRelatedLanes));
  ASSERT_NE(dut, nullptr);

  const auto traffic_lights = dut->TrafficLights();
  ASSERT_EQ(static_cast<int>(traffic_lights.size()), 1);

  const TrafficLight* tl = dut->GetTrafficLight(TrafficLight::Id("MyTrafficLight"));
  ASSERT_NE(tl, nullptr);

  const auto& related_lanes = tl->related_lanes();
  ASSERT_EQ(static_cast<int>(related_lanes.size()), 2);
  EXPECT_EQ(related_lanes[0], LaneId("lane_north"));
  EXPECT_EQ(related_lanes[1], LaneId("lane_south"));
}

GTEST_TEST(TrafficLightBookLoaderTest, LoadWithoutRelatedLanes) {
  const auto dut = LoadTrafficLightBook(std::string(kYamlWithoutRelatedLanes));
  ASSERT_NE(dut, nullptr);

  const TrafficLight* tl = dut->GetTrafficLight(TrafficLight::Id("MyTrafficLight"));
  ASSERT_NE(tl, nullptr);

  EXPECT_TRUE(tl->related_lanes().empty());
}

GTEST_TEST(TrafficLightBookLoaderTest, FindByLaneWithLoadedBook) {
  const auto dut = LoadTrafficLightBook(std::string(kYamlTwoTrafficLights));
  ASSERT_NE(dut, nullptr);

  // lane_1 is only in TrafficLightA.
  const auto result_lane_1 = dut->FindByLane(LaneId("lane_1"));
  ASSERT_EQ(static_cast<int>(result_lane_1.size()), 1);
  EXPECT_EQ(result_lane_1[0]->id(), TrafficLight::Id("TrafficLightA"));

  // lane_2 is in both.
  const auto result_lane_2 = dut->FindByLane(LaneId("lane_2"));
  EXPECT_EQ(static_cast<int>(result_lane_2.size()), 2);

  // lane_3 is only in TrafficLightB.
  const auto result_lane_3 = dut->FindByLane(LaneId("lane_3"));
  ASSERT_EQ(static_cast<int>(result_lane_3.size()), 1);
  EXPECT_EQ(result_lane_3[0]->id(), TrafficLight::Id("TrafficLightB"));

  // Unknown lane returns empty.
  EXPECT_TRUE(dut->FindByLane(LaneId("unknown_lane")).empty());
}

}  // namespace
}  // namespace maliput
