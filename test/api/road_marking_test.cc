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
#include "maliput/api/objects/road_marking.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/objects/road_object.h"
#include "maliput/common/error.h"
#include "maliput/math/bounding_box.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace api {
namespace objects {
namespace {

// -- RoadMarkingType tests --

GTEST_TEST(RoadMarkingTypeTest, DefaultValue) {
  RoadMarkingType dut{};
  EXPECT_EQ(dut, RoadMarkingType::kStop);
}

GTEST_TEST(RoadMarkingTypeTest, InstantiateAndAssign) {
  RoadMarkingType dut{};
  for (RoadMarkingType type :
       {RoadMarkingType::kStop, RoadMarkingType::kStopLine, RoadMarkingType::kCrosswalk, RoadMarkingType::kParkingSpace,
        RoadMarkingType::kEmergencyLane, RoadMarkingType::kSpeedLimit, RoadMarkingType::kDoNotStop,
        RoadMarkingType::kRailRoad, RoadMarkingType::kGiveWay, RoadMarkingType::kArrowTurnRight,
        RoadMarkingType::kArrowTurnLeft, RoadMarkingType::kArrowForwardTurnRight,
        RoadMarkingType::kArrowForwardTurnLeft, RoadMarkingType::kArrowForward,
        RoadMarkingType::kArrowForwardTurnRightTurnLeft, RoadMarkingType::kArrowTurnRightTurnLeft,
        RoadMarkingType::kArrowUTurnRight, RoadMarkingType::kArrowUTurnLeft, RoadMarkingType::kUnknown}) {
    dut = type;
    EXPECT_EQ(dut, type);
  }
}

GTEST_TEST(RoadMarkingTypeTest, MapperTest) {
  const auto dut = RoadMarkingTypeMapper();
  EXPECT_EQ(static_cast<int>(dut.size()), 19);

  // Verify all enum values are present.
  EXPECT_STREQ(dut.at(RoadMarkingType::kStop), "Stop");
  EXPECT_STREQ(dut.at(RoadMarkingType::kStopLine), "StopLine");
  EXPECT_STREQ(dut.at(RoadMarkingType::kCrosswalk), "Crosswalk");
  EXPECT_STREQ(dut.at(RoadMarkingType::kParkingSpace), "ParkingSpace");
  EXPECT_STREQ(dut.at(RoadMarkingType::kEmergencyLane), "EmergencyLane");
  EXPECT_STREQ(dut.at(RoadMarkingType::kSpeedLimit), "SpeedLimit");
  EXPECT_STREQ(dut.at(RoadMarkingType::kDoNotStop), "DoNotStop");
  EXPECT_STREQ(dut.at(RoadMarkingType::kRailRoad), "RailRoad");
  EXPECT_STREQ(dut.at(RoadMarkingType::kGiveWay), "GiveWay");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowTurnRight), "ArrowTurnRight");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowTurnLeft), "ArrowTurnLeft");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowForwardTurnRight), "ArrowForwardTurnRight");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowForwardTurnLeft), "ArrowForwardTurnLeft");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowForward), "ArrowForward");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowForwardTurnRightTurnLeft), "ArrowForwardTurnRightTurnLeft");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowTurnRightTurnLeft), "ArrowTurnRightTurnLeft");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowUTurnRight), "ArrowUTurnRight");
  EXPECT_STREQ(dut.at(RoadMarkingType::kArrowUTurnLeft), "ArrowUTurnLeft");
  EXPECT_STREQ(dut.at(RoadMarkingType::kUnknown), "Unknown");
}

GTEST_TEST(RoadMarkingTypeTest, HashWorks) {
  const std::hash<RoadMarkingType> hasher;
  EXPECT_NO_THROW(hasher(RoadMarkingType::kStop));
  EXPECT_NO_THROW(hasher(RoadMarkingType::kUnknown));
}

// -- RoadMarkingValueUnit tests --

GTEST_TEST(RoadMarkingValueUnitTest, MapperTest) {
  const auto dut = RoadMarkingValueUnitMapper();
  EXPECT_EQ(static_cast<int>(dut.size()), 3);
  EXPECT_STREQ(dut.at(RoadMarkingValueUnit::kMetersPerSecond), "m/s");
  EXPECT_STREQ(dut.at(RoadMarkingValueUnit::kKilometersPerHour), "km/h");
  EXPECT_STREQ(dut.at(RoadMarkingValueUnit::kMilesPerHour), "mph");
}

// -- RoadMarkingValue tests --

GTEST_TEST(RoadMarkingValueTest, DefaultConstruction) {
  RoadMarkingValue dut{};
  EXPECT_DOUBLE_EQ(dut.value, 0.);
  EXPECT_EQ(dut.unit, RoadMarkingValueUnit::kMetersPerSecond);
}

GTEST_TEST(RoadMarkingValueTest, Equality) {
  const RoadMarkingValue val1{60.0, RoadMarkingValueUnit::kKilometersPerHour};
  const RoadMarkingValue val2{60.0, RoadMarkingValueUnit::kKilometersPerHour};
  const RoadMarkingValue val3{30.0, RoadMarkingValueUnit::kKilometersPerHour};
  const RoadMarkingValue val4{60.0, RoadMarkingValueUnit::kMilesPerHour};

  EXPECT_EQ(val1, val2);
  EXPECT_NE(val1, val3);
  EXPECT_NE(val1, val4);
  EXPECT_NE(val3, val4);
}

// -- RoadMarking tests --

class RoadMarkingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const RoadMarking::Id id("marking_1");
    const RoadMarkingType type = RoadMarkingType::kCrosswalk;
    const RoadObjectPosition position(InertialPosition(10., 20., 0.), LaneId("lane_a"), LanePosition(5., 0., 0.));
    const Rotation orientation = Rotation::FromRpy(0., 0., 1.57);
    const maliput::math::BoundingBox bounding_box(math::Vector3(10., 20., 0.), math::Vector3(3., 5., 0.05),
                                                  math::RollPitchYaw(0., 0., 1.57), 0.01);
    std::vector<LaneId> related_lanes{LaneId("lane_a"), LaneId("lane_b"), LaneId("lane_c")};
    std::optional<std::string> name("Main Crosswalk");

    std::vector<std::unique_ptr<Outline>> outlines;
    std::vector<OutlineCorner> corners;
    corners.emplace_back(math::Vector3(8., 18., 0.));
    corners.emplace_back(math::Vector3(12., 18., 0.));
    corners.emplace_back(math::Vector3(12., 22., 0.));
    corners.emplace_back(math::Vector3(8., 22., 0.));
    outlines.push_back(std::make_unique<Outline>(Outline::Id("outline_crosswalk"), std::move(corners), true));

    dut_ = std::make_unique<RoadMarking>(id, type, position, orientation, bounding_box, std::move(related_lanes),
                                         std::move(name), std::move(outlines));
  }

  std::unique_ptr<RoadMarking> dut_;
};

TEST_F(RoadMarkingTest, Id) { EXPECT_EQ(dut_->id().string(), "marking_1"); }

TEST_F(RoadMarkingTest, Name) {
  ASSERT_TRUE(dut_->name().has_value());
  EXPECT_EQ(dut_->name().value(), "Main Crosswalk");
}

TEST_F(RoadMarkingTest, Type) { EXPECT_EQ(dut_->type(), RoadMarkingType::kCrosswalk); }

TEST_F(RoadMarkingTest, Position) {
  EXPECT_DOUBLE_EQ(dut_->position().inertial_position().x(), 10.);
  EXPECT_DOUBLE_EQ(dut_->position().inertial_position().y(), 20.);
  EXPECT_DOUBLE_EQ(dut_->position().inertial_position().z(), 0.);
  EXPECT_TRUE(dut_->position().has_lane_position());
  ASSERT_TRUE(dut_->position().lane_id().has_value());
  EXPECT_EQ(dut_->position().lane_id()->string(), "lane_a");
  ASSERT_TRUE(dut_->position().lane_position().has_value());
  EXPECT_DOUBLE_EQ(dut_->position().lane_position()->s(), 5.);
}

TEST_F(RoadMarkingTest, Orientation) {
  const auto& rot = dut_->orientation();
  EXPECT_DOUBLE_EQ(rot.rpy().roll_angle(), 0.);
  EXPECT_DOUBLE_EQ(rot.rpy().pitch_angle(), 0.);
  EXPECT_DOUBLE_EQ(rot.rpy().yaw_angle(), 1.57);
}

TEST_F(RoadMarkingTest, BoundingBox) {
  const auto& bb = dut_->bounding_box();
  EXPECT_DOUBLE_EQ(bb.position().x(), 10.);
  EXPECT_DOUBLE_EQ(bb.position().y(), 20.);
  EXPECT_DOUBLE_EQ(bb.position().z(), 0.);
  EXPECT_DOUBLE_EQ(bb.box_size().x(), 3.);
  EXPECT_DOUBLE_EQ(bb.box_size().y(), 5.);
  EXPECT_DOUBLE_EQ(bb.box_size().z(), 0.05);
}

TEST_F(RoadMarkingTest, RelatedLanes) {
  const auto& lanes = dut_->related_lanes();
  ASSERT_EQ(static_cast<int>(lanes.size()), 3);
  EXPECT_EQ(lanes[0].string(), "lane_a");
  EXPECT_EQ(lanes[1].string(), "lane_b");
  EXPECT_EQ(lanes[2].string(), "lane_c");
}

TEST_F(RoadMarkingTest, Outlines) {
  EXPECT_EQ(dut_->num_outlines(), 1);
  const auto& outlines = dut_->outlines();
  ASSERT_EQ(static_cast<int>(outlines.size()), 1);
  EXPECT_EQ(outlines[0]->id().string(), "outline_crosswalk");
  EXPECT_EQ(outlines[0]->num_corners(), 4);
  EXPECT_TRUE(outlines[0]->is_closed());
}

TEST_F(RoadMarkingTest, OutlineByIndex) {
  const Outline* outline = dut_->outline(0);
  ASSERT_NE(outline, nullptr);
  EXPECT_EQ(outline->id().string(), "outline_crosswalk");
}

TEST_F(RoadMarkingTest, OutlineOutOfRange) {
  EXPECT_THROW(dut_->outline(-1), maliput::common::assertion_error);
  EXPECT_THROW(dut_->outline(1), maliput::common::assertion_error);
}

TEST_F(RoadMarkingTest, GetValueNotSet) { EXPECT_FALSE(dut_->GetValue().has_value()); }

// -- RoadMarking with value --

GTEST_TEST(RoadMarkingWithValueTest, SpeedLimitMarking) {
  const RoadMarking::Id id("speed_60");
  const RoadObjectPosition position(InertialPosition(5., 10., 0.));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const maliput::math::BoundingBox bounding_box(math::Vector3(5., 10., 0.), math::Vector3(1., 2., 0.05),
                                                math::RollPitchYaw(0., 0., 0.), 0.01);
  const RoadMarkingValue value{60.0, RoadMarkingValueUnit::kKilometersPerHour};

  RoadMarking dut(id, RoadMarkingType::kSpeedLimit, position, orientation, bounding_box, {LaneId("lane_x")}, "60 km/h",
                  {}, value);

  EXPECT_EQ(dut.id().string(), "speed_60");
  EXPECT_EQ(dut.type(), RoadMarkingType::kSpeedLimit);
  ASSERT_TRUE(dut.name().has_value());
  EXPECT_EQ(dut.name().value(), "60 km/h");
  ASSERT_TRUE(dut.GetValue().has_value());
  EXPECT_DOUBLE_EQ(dut.GetValue()->value, 60.0);
  EXPECT_EQ(dut.GetValue()->unit, RoadMarkingValueUnit::kKilometersPerHour);
}

GTEST_TEST(RoadMarkingWithValueTest, MilesPerHour) {
  const RoadMarking::Id id("speed_35_mph");
  const RoadObjectPosition position(InertialPosition(0., 0., 0.));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const maliput::math::BoundingBox bounding_box(math::Vector3(0., 0., 0.), math::Vector3(1., 1., 0.05),
                                                math::RollPitchYaw(0., 0., 0.), 0.01);
  const RoadMarkingValue value{35.0, RoadMarkingValueUnit::kMilesPerHour};

  RoadMarking dut(id, RoadMarkingType::kSpeedLimit, position, orientation, bounding_box, {}, std::nullopt, {}, value);

  ASSERT_TRUE(dut.GetValue().has_value());
  EXPECT_DOUBLE_EQ(dut.GetValue()->value, 35.0);
  EXPECT_EQ(dut.GetValue()->unit, RoadMarkingValueUnit::kMilesPerHour);
}

// -- Minimal RoadMarking (no optional fields) --

GTEST_TEST(RoadMarkingMinimalTest, NoOptionalFields) {
  const RoadMarking::Id id("minimal");
  const RoadObjectPosition position(InertialPosition(0., 0., 0.));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const maliput::math::BoundingBox bounding_box(math::Vector3(0., 0., 0.), math::Vector3(1., 1., 0.05),
                                                math::RollPitchYaw(0., 0., 0.), 0.01);

  RoadMarking dut(id, RoadMarkingType::kArrowForward, position, orientation, bounding_box, {});

  EXPECT_EQ(dut.id().string(), "minimal");
  EXPECT_EQ(dut.type(), RoadMarkingType::kArrowForward);
  EXPECT_FALSE(dut.name().has_value());
  EXPECT_FALSE(dut.position().has_lane_position());
  EXPECT_TRUE(dut.related_lanes().empty());
  EXPECT_EQ(dut.num_outlines(), 0);
  EXPECT_FALSE(dut.GetValue().has_value());
}

// -- Multiple outlines --

GTEST_TEST(RoadMarkingOutlinesTest, MultipleOutlines) {
  const RoadMarking::Id id("multi_outline");
  const RoadObjectPosition position(InertialPosition(0., 0., 0.));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const maliput::math::BoundingBox bounding_box(math::Vector3(0., 0., 0.), math::Vector3(4., 4., 0.05),
                                                math::RollPitchYaw(0., 0., 0.), 0.01);

  std::vector<std::unique_ptr<Outline>> outlines;

  std::vector<OutlineCorner> corners1;
  corners1.emplace_back(math::Vector3(0., 0., 0.));
  corners1.emplace_back(math::Vector3(1., 0., 0.));
  corners1.emplace_back(math::Vector3(1., 1., 0.));
  outlines.push_back(std::make_unique<Outline>(Outline::Id("outline_0"), std::move(corners1), true));

  std::vector<OutlineCorner> corners2;
  corners2.emplace_back(math::Vector3(2., 0., 0.));
  corners2.emplace_back(math::Vector3(3., 0., 0.));
  corners2.emplace_back(math::Vector3(3., 1., 0.));
  corners2.emplace_back(math::Vector3(2., 1., 0.));
  outlines.push_back(std::make_unique<Outline>(Outline::Id("outline_1"), std::move(corners2), false));

  RoadMarking dut(id, RoadMarkingType::kCrosswalk, position, orientation, bounding_box, {}, std::nullopt,
                  std::move(outlines));

  EXPECT_EQ(dut.num_outlines(), 2);

  const Outline* o0 = dut.outline(0);
  ASSERT_NE(o0, nullptr);
  EXPECT_EQ(o0->id().string(), "outline_0");
  EXPECT_EQ(o0->num_corners(), 3);
  EXPECT_TRUE(o0->is_closed());

  const Outline* o1 = dut.outline(1);
  ASSERT_NE(o1, nullptr);
  EXPECT_EQ(o1->id().string(), "outline_1");
  EXPECT_EQ(o1->num_corners(), 4);
  EXPECT_FALSE(o1->is_closed());
}

}  // namespace
}  // namespace objects
}  // namespace api
}  // namespace maliput
