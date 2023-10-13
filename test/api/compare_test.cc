// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota.
// All rights reserved.
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
#include "maliput/api/compare.h"

#include <cmath>

#include <gtest/gtest.h>

#include "assert_compare.h"
#include "maliput/math/overlapping_type.h"

namespace maliput {
namespace api {
namespace test {
namespace {

static constexpr double kTolerance = 1e-3;

TEST(IsInertialPositionCloseTest, Test) {
  const InertialPosition test_value{0., 10., 10.};
  const InertialPosition close_pos{1.e-4, 10., 10.};
  EXPECT_EQ(std::nullopt, IsInertialPositionClose(test_value, close_pos, kTolerance).message);
  const InertialPosition non_close_pos{1.e-2, 10., 10.};
  const common::ComparisonResult<InertialPosition> res = IsInertialPositionClose(test_value, non_close_pos, kTolerance);
  EXPECT_NE(std::nullopt, res.message);
  const std::string expected_msg =
      "InertialPositions are different at x coordinate. pos1.x(): 0.000000 vs. pos2.x(): 0.010000, diff = 0.010000, "
      "tolerance = 0.001000\n";
  EXPECT_EQ(expected_msg, res.message.value());
}

TEST(IsLanePositionCloseTest, Test) {
  const LanePosition test_value{0., 10., 10.};
  const LanePosition close_pos{1.e-4, 10., 10.};
  EXPECT_EQ(std::nullopt, IsLanePositionClose(test_value, close_pos, kTolerance).message);
  const LanePosition non_close_pos{1.e-2, 10., 10.};
  const common::ComparisonResult<LanePosition> res = IsLanePositionClose(test_value, non_close_pos, kTolerance);
  EXPECT_NE(std::nullopt, res.message);
  const std::string expected_msg =
      "LanePositions are different at s coordinate. pos1.s(): 0.000000 vs. pos2.s(): 0.010000, diff = 0.010000, "
      "tolerance = 0.001000\n";
  EXPECT_EQ(expected_msg, res.message.value());
}

TEST(IsRotationCloseTest, Test) {
  const Rotation test_value{Rotation::FromRpy(0., 10., 10.)};
  const Rotation close_rot{Rotation::FromRpy(1.e-4, 10., 10.)};
  EXPECT_EQ(std::nullopt, IsRotationClose(test_value, close_rot, kTolerance).message);
  const Rotation non_close_rot{Rotation::FromRpy(1.e-2, 10., 10.)};
  const common::ComparisonResult<Rotation> res = IsRotationClose(test_value, non_close_rot, kTolerance);
  EXPECT_NE(std::nullopt, res.message);
  const std::string expected_msg =
      "Rotations are different at roll angle. rot1.roll(): -3.141593 vs. rot2.roll(): -3.131593, diff = 0.010000, "
      "tolerance = 0.001000\n";
  EXPECT_EQ(expected_msg, res.message.value());
}

TEST(IsRBoundsCloseTest, Test) {
  const RBounds test_value{0., 10.};
  const RBounds close_r_bounds{-1.e-4, 10.};
  EXPECT_EQ(std::nullopt, IsRBoundsClose(test_value, close_r_bounds, kTolerance).message);
  const RBounds close_r_max{-1.e-2, 10.};
  common::ComparisonResult<RBounds> res = IsRBoundsClose(test_value, close_r_max, kTolerance);
  std::string expected_msg =
      "RBounds are different at r_max. rbounds1.r_max: 10.000000 vs. rbounds2.r_max: 100.000000, diff = 90.000000, "
      "tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  const RBounds close_r_min{-1e-4, 1.e-2};
  res = IsRBoundsClose(test_value, close_r_min, kTolerance);
  expected_msg =
      "RBounds are different at r_max. rbounds1.r_max: 10.000000 vs. rbounds2.r_max: 0.010000, diff = 9.990000, "
      "tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());
  const RBounds non_close_bounds{-1.e-2, 100.};
  res = IsRBoundsClose(test_value, non_close_bounds, kTolerance);
  expected_msg =
      "RBounds are different at r_min. rbounds1.r_min: 0.000000 vs. rbounds2.r_min: -0.010000, diff = 0.010000, "
      "tolerance = 0.001000\nRBounds are different at r_max. rbounds1.r_max: 10.000000 vs. rbounds2.r_max: 100.000000, "
      "diff = 90.000000, tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());
}

TEST(IsHBoundsCloseTest, Test) {
  const HBounds test_value{0., 10.};
  const HBounds close_h_bounds{-1.e-4, 10.};
  EXPECT_EQ(std::nullopt, IsHBoundsClose(test_value, close_h_bounds, kTolerance).message);
  const HBounds close_h_max{-1.e-2, 10.};
  common::ComparisonResult<HBounds> res = IsHBoundsClose(test_value, close_h_max, kTolerance);
  std::string expected_msg =
      "HBounds are different at min. rbounds1.min(): 0.000000 vs. rbounds2.min(): -0.010000, diff = 0.010000, "
      "tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());
  const HBounds close_h_min{-1.e-4, 1.e-2};
  res = IsHBoundsClose(test_value, close_h_min, kTolerance);
  expected_msg =
      "HBounds are different at max. rbounds1.max(): 10.000000 vs. rbounds2.max(): 0.010000, diff = 9.990000, "
      "tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());
  const HBounds non_close_bounds{-1.e-2, 100.};
  res = IsHBoundsClose(test_value, non_close_bounds, kTolerance);
  expected_msg =
      "HBounds are different at min. rbounds1.min(): 0.000000 vs. rbounds2.min(): -0.010000, diff = 0.010000, "
      "tolerance = 0.001000\nHBounds are different at max. rbounds1.max(): 10.000000 vs. rbounds2.max(): 100.000000, "
      "diff = 90.000000, tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());
}

TEST(IsLanePositionResultCloseTest, Test) {
  const InertialPosition inertial_pos{0., 10., 10.};
  const LanePosition lane_pos{0., 10., 10.};
  const double distance{10.};
  const LanePositionResult test_value{lane_pos, inertial_pos, distance};
  const LanePositionResult close_value{lane_pos, inertial_pos, distance};
  EXPECT_EQ(std::nullopt, IsLanePositionResultClose(test_value, close_value, kTolerance).message);

  // Non close per lane position.
  const LanePositionResult non_lane_pos_close{{10., 10., 10.}, inertial_pos, distance};
  common::ComparisonResult<LanePositionResult> res =
      IsLanePositionResultClose(test_value, non_lane_pos_close, kTolerance);
  std::string expected_msg =
      "LanePositions are different at s coordinate. pos1.s(): 0.000000 vs. pos2.s(): 10.000000, diff = 10.000000, "
      "tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());

  // Non close per nearest position.
  const LanePositionResult non_inertial_pos_close{lane_pos, {10., 10., 10.}, distance};
  res = IsLanePositionResultClose(test_value, non_inertial_pos_close, kTolerance);
  expected_msg =
      "InertialPositions are different at x coordinate. pos1.x(): 0.000000 vs. pos2.x(): 10.000000, diff = 10.000000, "
      "tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());

  // Non close per distance.
  const LanePositionResult non_distance_close{lane_pos, inertial_pos, 85.};
  res = IsLanePositionResultClose(test_value, non_distance_close, kTolerance);
  expected_msg =
      "LanePositionResult are different at distance. lpr_a.distance: 10.000000 vs. lpr_b.distance: 85.000000, diff = "
      "75.000000, tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());
}

TEST(IsRoadPositionResultCloseTest, Test) {
  const InertialPosition inertial_pos{0., 10., 10.};
  const LanePosition lane_pos{0., 10., 10.};
  const RoadPosition road_pos{reinterpret_cast<const Lane*>(0xDeadBeef), lane_pos};
  const double distance{10.};
  const RoadPositionResult test_value{road_pos, inertial_pos, distance};
  const RoadPositionResult close_value{road_pos, inertial_pos, distance};
  EXPECT_EQ(std::nullopt, IsRoadPositionResultClose(test_value, close_value, kTolerance).message);

  // Non close per non lane matching.
  const RoadPositionResult non_lane_matching{
      {reinterpret_cast<const Lane*>(0xDeadD00d), lane_pos}, inertial_pos, distance};
  // TODO(francocipollone): Improve this test by mocking the entire Lane and returning two different lane ids.
  //                       Here the pointers are invented to make the comparison to fail, however getting the id isn't
  //                       allowed.
  EXPECT_DEATH(IsRoadPositionResultClose(test_value, non_lane_matching, kTolerance), "");

  // Non close per lane position.
  const RoadPositionResult non_lane_pos_close{{road_pos.lane, {10., 10., 10.}}, inertial_pos, distance};
  common::ComparisonResult<RoadPositionResult> res =
      IsRoadPositionResultClose(test_value, non_lane_pos_close, kTolerance);
  std::string expected_msg =
      "LanePositions are different at s coordinate. pos1.s(): 0.000000 vs. pos2.s(): 10.000000, diff = 10.000000, "
      "tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());

  // Non close per nearest position.
  const RoadPositionResult non_inertial_pos_close{road_pos, {10., 10., 10.}, distance};
  res = IsRoadPositionResultClose(test_value, non_inertial_pos_close, kTolerance);
  expected_msg =
      "InertialPositions are different at x coordinate. pos1.x(): 0.000000 vs. pos2.x(): 10.000000, diff = 10.000000, "
      "tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());

  // Non close per distance.
  const RoadPositionResult non_distance_close{road_pos, inertial_pos, 85.};
  res = IsRoadPositionResultClose(test_value, non_distance_close, kTolerance);
  expected_msg =
      "LanePositionResult are different at distance. lpr_a.distance: 10.000000 vs. lpr_b.distance: 85.000000, diff = "
      "75.000000, tolerance = 0.001000\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());
}

TEST(IsLaneEndEqualTest, Test) {
  const LaneEnd lane_end1(reinterpret_cast<const Lane*>(0xDeadBeef), maliput::api::LaneEnd::Which::kStart);
  const LaneEnd lane_end2(reinterpret_cast<const Lane*>(0xDeadD00d), maliput::api::LaneEnd::Which::kStart);
  const LaneEnd lane_end3(reinterpret_cast<const Lane*>(0xDeadBeef), maliput::api::LaneEnd::Which::kFinish);

  EXPECT_EQ(std::nullopt, IsLaneEndEqual(lane_end1, lane_end1).message);
  common::ComparisonResult<LaneEnd> res = IsLaneEndEqual(lane_end1, lane_end3);
  const std::string expected_msg =
      "lane_end1.end is different from lane_end2.end. lane_end1.end: kStart vs. lane_end2.end: kFinish\n";
  EXPECT_NE(std::nullopt, res.message);
  EXPECT_EQ(expected_msg, res.message.value());

  // TODO(francocipollone): Improve this test by mocking the entire Lane and returning two different lane ids.
  //                       Here the pointers are invented to make the comparison to fail, however getting the id isn't
  //                       allowed.
  EXPECT_DEATH(IsLaneEndEqual(lane_end1, lane_end2), "");
}

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace maliput
