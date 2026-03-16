// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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
/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/traffic_sign.h"
/* clang-format on */

#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/math/bounding_box.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

GTEST_TEST(TrafficSignTypeTest, InstantiateAndAssign) {
  TrafficSignType dut{};
  EXPECT_EQ(dut, TrafficSignType::kStop);
  for (TrafficSignType type :
       {TrafficSignType::kYield, TrafficSignType::kSpeedLimit, TrafficSignType::kNoEntry, TrafficSignType::kOneWay,
        TrafficSignType::kPedestrianCrossing, TrafficSignType::kNoLeftTurn, TrafficSignType::kNoRightTurn,
        TrafficSignType::kNoUTurn, TrafficSignType::kSchoolZone, TrafficSignType::kConstruction,
        TrafficSignType::kRailroadCrossing}) {
    EXPECT_NE(dut, type);
    dut = type;
    EXPECT_EQ(dut, type);
  }
}

GTEST_TEST(TrafficSignTypeTest, MapperTest) {
  const auto dut = TrafficSignTypeMapper();
  const std::vector<TrafficSignType> expected_types{
      TrafficSignType::kStop,       TrafficSignType::kYield,        TrafficSignType::kSpeedLimit,
      TrafficSignType::kNoEntry,    TrafficSignType::kOneWay,       TrafficSignType::kPedestrianCrossing,
      TrafficSignType::kNoLeftTurn, TrafficSignType::kNoRightTurn,  TrafficSignType::kNoUTurn,
      TrafficSignType::kSchoolZone, TrafficSignType::kConstruction, TrafficSignType::kRailroadCrossing,
  };
  EXPECT_EQ(dut.size(), expected_types.size());
  for (TrafficSignType type : expected_types) {
    EXPECT_EQ(static_cast<int>(dut.count(type)), 1);
  }
}

GTEST_TEST(TrafficSignTest, Constructor) {
  const TrafficSign::Id kId("test_stop_sign");
  const TrafficSignType kType = TrafficSignType::kStop;
  const InertialPosition kPosition(1., 2., 3.);
  const Rotation kOrientation = Rotation::FromRpy(0., 0., 1.57);
  const std::optional<std::string> kMessage = std::nullopt;
  const std::vector<LaneId> kRelatedLanes{LaneId("lane_1"), LaneId("lane_2")};
  const maliput::math::BoundingBox kBoundingBox(maliput::math::Vector3(0., 0., 0.),
                                                maliput::math::Vector3(0.05, 0.762, 0.762),
                                                maliput::math::RollPitchYaw(0., 0., 0.), 1e-3);

  const TrafficSign dut(kId, kType, kPosition, kOrientation, kMessage, kRelatedLanes, kBoundingBox);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type(), kType);
  EXPECT_EQ(dut.position_road_network().x(), kPosition.x());
  EXPECT_EQ(dut.position_road_network().y(), kPosition.y());
  EXPECT_EQ(dut.position_road_network().z(), kPosition.z());
  EXPECT_EQ(dut.message(), kMessage);
  EXPECT_EQ(dut.related_lanes().size(), 2u);
  EXPECT_EQ(dut.related_lanes()[0], LaneId("lane_1"));
  EXPECT_EQ(dut.related_lanes()[1], LaneId("lane_2"));
  EXPECT_DOUBLE_EQ(dut.bounding_box().box_size().x(), 0.05);
  EXPECT_DOUBLE_EQ(dut.bounding_box().box_size().y(), 0.762);
  EXPECT_DOUBLE_EQ(dut.bounding_box().box_size().z(), 0.762);
}

GTEST_TEST(TrafficSignTest, ConstructorWithMessage) {
  const TrafficSign::Id kId("speed_limit_60");
  const TrafficSignType kType = TrafficSignType::kSpeedLimit;
  const InertialPosition kPosition(10., 20., 3.);
  const Rotation kOrientation = Rotation::FromRpy(0., 0., 0.);
  const std::optional<std::string> kMessage = "60";
  const maliput::math::BoundingBox kBoundingBox(maliput::math::Vector3(0., 0., 0.),
                                                maliput::math::Vector3(0.05, 0.762, 0.762),
                                                maliput::math::RollPitchYaw(0., 0., 0.), 1e-3);

  const TrafficSign dut(kId, kType, kPosition, kOrientation, kMessage, {}, kBoundingBox);

  EXPECT_EQ(dut.id(), kId);
  EXPECT_EQ(dut.type(), kType);
  EXPECT_TRUE(dut.message().has_value());
  EXPECT_EQ(dut.message().value(), "60");
  EXPECT_TRUE(dut.related_lanes().empty());
}

GTEST_TEST(TrafficSignTest, ConstructorWithCustomBoundingBox) {
  const TrafficSign::Id kId("big_sign");
  const TrafficSignType kType = TrafficSignType::kConstruction;
  const InertialPosition kPosition(5., 10., 2.);
  const Rotation kOrientation = Rotation::FromRpy(0., 0., 0.);
  const maliput::math::BoundingBox kBoundingBox(maliput::math::Vector3(0., 0., 0.),
                                                maliput::math::Vector3(0.1, 1.2, 1.2),
                                                maliput::math::RollPitchYaw(0., 0., 0.), 1e-3);

  const TrafficSign dut(kId, kType, kPosition, kOrientation, std::nullopt, {}, kBoundingBox);

  const auto& bb = dut.bounding_box();
  EXPECT_DOUBLE_EQ(bb.box_size().x(), 0.1);
  EXPECT_DOUBLE_EQ(bb.box_size().y(), 1.2);
  EXPECT_DOUBLE_EQ(bb.box_size().z(), 1.2);
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
