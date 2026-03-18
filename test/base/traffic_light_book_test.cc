// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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
#include "maliput/base/traffic_light_book.h"

#include <gtest/gtest.h>

#include "assert_compare.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/rules/compare.h"

namespace maliput {
namespace {

using api::LaneId;
using api::rules::BulbGroup;
using api::rules::TrafficLight;
using maliput::test::AssertCompare;

GTEST_TEST(TrafficLightBookTest, BasicTest) {
  const TrafficLight::Id id("my traffic light");
  const std::vector<LaneId> related_lanes{LaneId("lane_1"), LaneId("lane_2")};
  std::vector<std::unique_ptr<BulbGroup>> empty_bulb_group{};
  auto traffic_light =
      std::make_unique<const TrafficLight>(id, api::InertialPosition(10, 11, 12), api::Rotation::FromRpy(1, 2, 3),
                                           std::move(empty_bulb_group), related_lanes);
  const TrafficLight* traffic_light_ptr = traffic_light.get();

  TrafficLightBook dut;

  const std::vector<const TrafficLight*> empty = dut.TrafficLights();
  EXPECT_EQ(static_cast<int>(empty.size()), 0);

  dut.AddTrafficLight(std::move(traffic_light));
  EXPECT_EQ(dut.GetTrafficLight(TrafficLight::Id("unknown_traffic light")), nullptr);
  EXPECT_TRUE(AssertCompare(IsEqual(dut.GetTrafficLight(id), traffic_light_ptr)));

  const std::vector<const TrafficLight*> nonempty = dut.TrafficLights();
  EXPECT_EQ(static_cast<int>(nonempty.size()), 1);
  EXPECT_TRUE(AssertCompare(IsEqual(nonempty.at(0), traffic_light_ptr)));
}

GTEST_TEST(TrafficLightBookTest, FindByLane) {
  const TrafficLight::Id id_a("tl_a");
  const TrafficLight::Id id_b("tl_b");
  const LaneId lane_1("lane_1");
  const LaneId lane_2("lane_2");
  const LaneId lane_unknown("lane_unknown");

  std::vector<std::unique_ptr<BulbGroup>> bg_a{};
  auto tl_a =
      std::make_unique<const TrafficLight>(id_a, api::InertialPosition(0, 0, 0), api::Rotation::FromRpy(0, 0, 0),
                                           std::move(bg_a), std::vector<LaneId>{lane_1, lane_2});

  std::vector<std::unique_ptr<BulbGroup>> bg_b{};
  auto tl_b =
      std::make_unique<const TrafficLight>(id_b, api::InertialPosition(1, 1, 1), api::Rotation::FromRpy(0, 0, 0),
                                           std::move(bg_b), std::vector<LaneId>{lane_2});

  TrafficLightBook dut;
  dut.AddTrafficLight(std::move(tl_a));
  dut.AddTrafficLight(std::move(tl_b));

  // lane_1 is related only to tl_a.
  const auto result_lane_1 = dut.FindByLane(lane_1);
  ASSERT_EQ(static_cast<int>(result_lane_1.size()), 1);
  EXPECT_EQ(result_lane_1[0]->id(), id_a);

  // lane_2 is related to both tl_a and tl_b.
  const auto result_lane_2 = dut.FindByLane(lane_2);
  EXPECT_EQ(static_cast<int>(result_lane_2.size()), 2);

  // Unknown lane returns empty.
  const auto result_unknown = dut.FindByLane(lane_unknown);
  EXPECT_TRUE(result_unknown.empty());
}

}  // namespace
}  // namespace maliput
