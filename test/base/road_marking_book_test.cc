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
#include "maliput/base/road_marking_book.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/objects/road_marking.h"
#include "maliput/api/objects/road_object.h"
#include "maliput/math/bounding_box.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace {

using api::InertialPosition;
using api::LaneId;
using api::Rotation;
using api::objects::Outline;
using api::objects::OutlineCorner;
using api::objects::RoadMarking;
using api::objects::RoadMarkingType;
using api::objects::RoadMarkingValue;
using api::objects::RoadMarkingValueUnit;
using api::objects::RoadObjectPosition;

// Helper to create a RoadMarking with a given ID, type, position, and related lanes.
std::unique_ptr<RoadMarking> MakeRoadMarking(const std::string& id, RoadMarkingType type, double x, double y, double z,
                                             std::vector<LaneId> related_lanes = {},
                                             std::optional<std::string> name = std::nullopt,
                                             std::optional<RoadMarkingValue> value = std::nullopt) {
  const RoadObjectPosition position(InertialPosition(x, y, z));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const math::BoundingBox bounding_box(math::Vector3(x, y, z), math::Vector3(1., 1., 0.05),
                                       math::RollPitchYaw(0., 0., 0.), 0.01);
  return std::make_unique<RoadMarking>(RoadMarking::Id(id), type, position, orientation, bounding_box,
                                       std::move(related_lanes), std::move(name),
                                       std::vector<std::unique_ptr<Outline>>{}, value);
}

// -- Empty book tests --

GTEST_TEST(RoadMarkingBookTest, EmptyBook) {
  RoadMarkingBook dut;

  EXPECT_TRUE(dut.RoadMarkings().empty());
  EXPECT_EQ(dut.GetRoadMarking(RoadMarking::Id("nonexistent")), nullptr);
  EXPECT_TRUE(dut.FindByType(RoadMarkingType::kStop).empty());
  EXPECT_TRUE(dut.FindByLane(LaneId("lane_1")).empty());
}

// -- AddRoadMarking and GetRoadMarking --

GTEST_TEST(RoadMarkingBookTest, AddAndGet) {
  RoadMarkingBook dut;

  auto marking = MakeRoadMarking("marking_1", RoadMarkingType::kCrosswalk, 10., 20., 0.);
  const RoadMarking* marking_ptr = marking.get();
  dut.AddRoadMarking(std::move(marking));

  const RoadMarking* result = dut.GetRoadMarking(RoadMarking::Id("marking_1"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, marking_ptr);
  EXPECT_EQ(result->id().string(), "marking_1");
  EXPECT_EQ(result->type(), RoadMarkingType::kCrosswalk);
}

GTEST_TEST(RoadMarkingBookTest, GetNonexistent) {
  RoadMarkingBook dut;

  dut.AddRoadMarking(MakeRoadMarking("marking_1", RoadMarkingType::kStop, 0., 0., 0.));
  EXPECT_EQ(dut.GetRoadMarking(RoadMarking::Id("marking_2")), nullptr);
}

GTEST_TEST(RoadMarkingBookTest, DuplicateIdThrows) {
  RoadMarkingBook dut;

  dut.AddRoadMarking(MakeRoadMarking("marking_1", RoadMarkingType::kStop, 0., 0., 0.));
  EXPECT_THROW(dut.AddRoadMarking(MakeRoadMarking("marking_1", RoadMarkingType::kCrosswalk, 1., 1., 0.)),
               std::logic_error);
}

GTEST_TEST(RoadMarkingBookTest, NullptrThrows) {
  RoadMarkingBook dut;

  EXPECT_THROW(dut.AddRoadMarking(nullptr), std::exception);
}

// -- RoadMarkings (list all) --

GTEST_TEST(RoadMarkingBookTest, RoadMarkingsList) {
  RoadMarkingBook dut;

  dut.AddRoadMarking(MakeRoadMarking("marking_1", RoadMarkingType::kStop, 0., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("marking_2", RoadMarkingType::kCrosswalk, 1., 1., 0.));
  dut.AddRoadMarking(MakeRoadMarking("marking_3", RoadMarkingType::kPrescribedStraight, 2., 2., 0.));

  const auto result = dut.RoadMarkings();
  EXPECT_EQ(static_cast<int>(result.size()), 3);
}

// -- FindByType --

GTEST_TEST(RoadMarkingBookTest, FindByType) {
  RoadMarkingBook dut;

  dut.AddRoadMarking(MakeRoadMarking("stop_1", RoadMarkingType::kStop, 0., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("stop_2", RoadMarkingType::kStop, 1., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("crosswalk_1", RoadMarkingType::kCrosswalk, 2., 0., 0.));

  const auto stops = dut.FindByType(RoadMarkingType::kStop);
  EXPECT_EQ(static_cast<int>(stops.size()), 2);

  const auto crosswalks = dut.FindByType(RoadMarkingType::kCrosswalk);
  EXPECT_EQ(static_cast<int>(crosswalks.size()), 1);
  EXPECT_EQ(crosswalks[0]->id().string(), "crosswalk_1");

  const auto arrows = dut.FindByType(RoadMarkingType::kPrescribedStraight);
  EXPECT_TRUE(arrows.empty());
}

GTEST_TEST(RoadMarkingBookTest, FindByTypeAllTypes) {
  RoadMarkingBook dut;

  // Add one marking for each road-marking-specific type.
  dut.AddRoadMarking(MakeRoadMarking("m_stop", RoadMarkingType::kStop, 0., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_stopline", RoadMarkingType::kStopLine, 1., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_crosswalk", RoadMarkingType::kCrosswalk, 2., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_parking", RoadMarkingType::kCarParking, 3., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_emergency", RoadMarkingType::kEmergencyLane, 4., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_speedlimit", RoadMarkingType::kSpeedLimit, 5., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_donotstop", RoadMarkingType::kNoStopping, 6., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_railroad", RoadMarkingType::kRailroadCrossing, 7., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_giveway", RoadMarkingType::kGiveWay, 8., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_arrow_tr", RoadMarkingType::kPrescribedRightTurn, 9., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_arrow_tl", RoadMarkingType::kPrescribedLeftTurn, 10., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_arrow_ftr", RoadMarkingType::kPrescribedRightTurnAndStraight, 11., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_arrow_ftl", RoadMarkingType::kPrescribedLeftTurnAndStraight, 12., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_arrow_f", RoadMarkingType::kPrescribedStraight, 13., 0., 0.));
  dut.AddRoadMarking(
      MakeRoadMarking("m_arrow_ftrtl", RoadMarkingType::kPrescribedLeftTurnRightTurnAndStraight, 14., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_arrow_trtl", RoadMarkingType::kPrescribedLeftTurnAndRightTurn, 15., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_arrow_utr", RoadMarkingType::kPrescribedUTurnRight, 16., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_arrow_utl", RoadMarkingType::kPrescribedUTurnLeft, 17., 0., 0.));
  dut.AddRoadMarking(MakeRoadMarking("m_unknown", RoadMarkingType::kUnknown, 18., 0., 0.));

  const auto all = dut.RoadMarkings();
  EXPECT_EQ(static_cast<int>(all.size()), 19);

  // Spot check a few types.
  EXPECT_EQ(static_cast<int>(dut.FindByType(RoadMarkingType::kSpeedLimit).size()), 1);
  EXPECT_EQ(static_cast<int>(dut.FindByType(RoadMarkingType::kPrescribedUTurnLeft).size()), 1);
  EXPECT_EQ(static_cast<int>(dut.FindByType(RoadMarkingType::kUnknown).size()), 1);
}

// -- FindByLane --

GTEST_TEST(RoadMarkingBookTest, FindByLane) {
  RoadMarkingBook dut;

  dut.AddRoadMarking(MakeRoadMarking("marking_1", RoadMarkingType::kStop, 0., 0., 0., {LaneId("lane_a")}));
  dut.AddRoadMarking(
      MakeRoadMarking("marking_2", RoadMarkingType::kCrosswalk, 1., 0., 0., {LaneId("lane_a"), LaneId("lane_b")}));
  dut.AddRoadMarking(
      MakeRoadMarking("marking_3", RoadMarkingType::kPrescribedStraight, 2., 0., 0., {LaneId("lane_c")}));

  const auto lane_a_markings = dut.FindByLane(LaneId("lane_a"));
  EXPECT_EQ(static_cast<int>(lane_a_markings.size()), 2);

  const auto lane_b_markings = dut.FindByLane(LaneId("lane_b"));
  EXPECT_EQ(static_cast<int>(lane_b_markings.size()), 1);
  EXPECT_EQ(lane_b_markings[0]->id().string(), "marking_2");

  const auto lane_c_markings = dut.FindByLane(LaneId("lane_c"));
  EXPECT_EQ(static_cast<int>(lane_c_markings.size()), 1);
  EXPECT_EQ(lane_c_markings[0]->id().string(), "marking_3");

  const auto lane_d_markings = dut.FindByLane(LaneId("lane_d"));
  EXPECT_TRUE(lane_d_markings.empty());
}

// -- GetValue --

GTEST_TEST(RoadMarkingBookTest, MarkingWithValue) {
  RoadMarkingBook dut;

  const RoadMarkingValue speed_limit{60.0, RoadMarkingValueUnit::kKilometersPerHour};
  dut.AddRoadMarking(MakeRoadMarking("speed_60", RoadMarkingType::kSpeedLimit, 5., 0., 0., {LaneId("lane_a")},
                                     "60 km/h", speed_limit));

  const RoadMarking* result = dut.GetRoadMarking(RoadMarking::Id("speed_60"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->type(), RoadMarkingType::kSpeedLimit);
  EXPECT_TRUE(result->name().has_value());
  EXPECT_EQ(result->name().value(), "60 km/h");

  ASSERT_TRUE(result->GetValue().has_value());
  EXPECT_DOUBLE_EQ(result->GetValue()->value, 60.0);
  EXPECT_EQ(result->GetValue()->unit, RoadMarkingValueUnit::kKilometersPerHour);
}

GTEST_TEST(RoadMarkingBookTest, MarkingWithoutValue) {
  RoadMarkingBook dut;

  dut.AddRoadMarking(MakeRoadMarking("stop_1", RoadMarkingType::kStop, 0., 0., 0.));

  const RoadMarking* result = dut.GetRoadMarking(RoadMarking::Id("stop_1"));
  ASSERT_NE(result, nullptr);
  EXPECT_FALSE(result->GetValue().has_value());
}

GTEST_TEST(RoadMarkingBookTest, MarkingValueEquality) {
  const RoadMarkingValue val1{60.0, RoadMarkingValueUnit::kKilometersPerHour};
  const RoadMarkingValue val2{60.0, RoadMarkingValueUnit::kKilometersPerHour};
  const RoadMarkingValue val3{30.0, RoadMarkingValueUnit::kMilesPerHour};
  const RoadMarkingValue val4{60.0, RoadMarkingValueUnit::kMilesPerHour};

  EXPECT_EQ(val1, val2);
  EXPECT_NE(val1, val3);
  EXPECT_NE(val1, val4);
}

// -- RoadMarking properties --

GTEST_TEST(RoadMarkingBookTest, MarkingProperties) {
  RoadMarkingBook dut;

  auto marking = MakeRoadMarking("marking_1", RoadMarkingType::kCrosswalk, 10., 20., 0., {LaneId("lane_a")},
                                 "Main Street Crosswalk");
  const RoadMarking* marking_ptr = marking.get();
  dut.AddRoadMarking(std::move(marking));

  const RoadMarking* result = dut.GetRoadMarking(RoadMarking::Id("marking_1"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, marking_ptr);
  EXPECT_EQ(result->id().string(), "marking_1");
  EXPECT_EQ(result->type(), RoadMarkingType::kCrosswalk);
  EXPECT_TRUE(result->name().has_value());
  EXPECT_EQ(result->name().value(), "Main Street Crosswalk");
  EXPECT_EQ(result->position().inertial_position().x(), 10.);
  EXPECT_EQ(result->position().inertial_position().y(), 20.);
  EXPECT_EQ(result->position().inertial_position().z(), 0.);
  EXPECT_EQ(static_cast<int>(result->related_lanes().size()), 1);
  EXPECT_EQ(result->related_lanes()[0], LaneId("lane_a"));
  EXPECT_EQ(result->num_outlines(), 0);
  EXPECT_FALSE(result->GetValue().has_value());
}

// -- Outlines --

GTEST_TEST(RoadMarkingBookTest, MarkingWithOutlines) {
  const RoadObjectPosition position(InertialPosition(5., 5., 0.));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const math::BoundingBox bounding_box(math::Vector3(5., 5., 0.), math::Vector3(2., 2., 0.05),
                                       math::RollPitchYaw(0., 0., 0.), 0.01);

  std::vector<OutlineCorner> corners;
  corners.emplace_back(math::Vector3(4., 4., 0.));
  corners.emplace_back(math::Vector3(6., 4., 0.));
  corners.emplace_back(math::Vector3(6., 6., 0.));
  corners.emplace_back(math::Vector3(4., 6., 0.));

  std::vector<std::unique_ptr<Outline>> outlines;
  outlines.push_back(std::make_unique<Outline>(Outline::Id("outline_0"), std::move(corners), true));

  auto marking = std::make_unique<RoadMarking>(RoadMarking::Id("crosswalk_outline"), RoadMarkingType::kCrosswalk,
                                               position, orientation, bounding_box, std::vector<LaneId>{}, std::nullopt,
                                               std::move(outlines));

  RoadMarkingBook dut;
  dut.AddRoadMarking(std::move(marking));

  const RoadMarking* result = dut.GetRoadMarking(RoadMarking::Id("crosswalk_outline"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->num_outlines(), 1);

  const Outline* outline = result->outline(0);
  ASSERT_NE(outline, nullptr);
  EXPECT_EQ(outline->id().string(), "outline_0");
  EXPECT_TRUE(outline->is_closed());
  EXPECT_EQ(outline->num_corners(), 4);
  EXPECT_DOUBLE_EQ(outline->corners()[0].position().x(), 4.);
  EXPECT_DOUBLE_EQ(outline->corners()[0].position().y(), 4.);
}

// -- Type and Unit Mappers --

GTEST_TEST(RoadMarkingBookTest, TypeMapper) {  // THIS
  const auto mapper = api::objects::RoadMarkingTypeMapper();
  EXPECT_GE(static_cast<int>(mapper.size()), 19);
  EXPECT_EQ(static_cast<int>(mapper.size()), static_cast<int>(RoadMarkingType::kUnknown) + 1);
  EXPECT_STREQ(mapper.at(RoadMarkingType::kStop), "Stop");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kStopLine), "StopLine");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kCrosswalk), "Crosswalk");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kCarParking), "CarParking");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kEmergencyLane), "EmergencyLane");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kSpeedLimit), "SpeedLimit");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kNoStopping), "NoStopping");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kRailroadCrossing), "RailroadCrossing");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kGiveWay), "GiveWay");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedRightTurn), "PrescribedRightTurn");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedLeftTurn), "PrescribedLeftTurn");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedRightTurnAndStraight), "PrescribedRightTurnAndStraight");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedLeftTurnAndStraight), "PrescribedLeftTurnAndStraight");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedStraight), "PrescribedStraight");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedLeftTurnRightTurnAndStraight),
               "PrescribedLeftTurnRightTurnAndStraight");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedLeftTurnAndRightTurn), "PrescribedLeftTurnAndRightTurn");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedUTurnRight), "PrescribedUTurnRight");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kPrescribedUTurnLeft), "PrescribedUTurnLeft");
  EXPECT_STREQ(mapper.at(RoadMarkingType::kUnknown), "Unknown");
}

GTEST_TEST(RoadMarkingBookTest, ValueUnitMapper) {
  const auto mapper = api::objects::RoadMarkingValueUnitMapper();
  EXPECT_EQ(static_cast<int>(mapper.size()), 3);
  EXPECT_STREQ(mapper.at(RoadMarkingValueUnit::kMetersPerSecond), "m/s");
  EXPECT_STREQ(mapper.at(RoadMarkingValueUnit::kKilometersPerHour), "km/h");
  EXPECT_STREQ(mapper.at(RoadMarkingValueUnit::kMilesPerHour), "mph");
}

}  // namespace
}  // namespace maliput
