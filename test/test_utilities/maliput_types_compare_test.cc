// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
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
#include "maliput/test_utilities/maliput_types_compare.h"

#include <cmath>

#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>

#include "maliput/math/overlapping_type.h"

namespace maliput {
namespace api {
namespace test {
namespace {

static constexpr double kTolerance = 1e-3;

TEST(IsInertialPositionCloseTest, Test) {
  const InertialPosition test_value{0., 10., 10.};
  const InertialPosition close_pos{1.e-4, 10., 10.};
  EXPECT_EQ(testing::AssertionSuccess(), IsInertialPositionClose(test_value, close_pos, kTolerance));
  const InertialPosition non_close_pos{1.e-2, 10., 10.};
  EXPECT_EQ(testing::AssertionFailure(), IsInertialPositionClose(test_value, non_close_pos, kTolerance));
}

TEST(IsLanePositionCloseTest, Test) {
  const LanePosition test_value{0., 10., 10.};
  const LanePosition close_pos{1.e-4, 10., 10.};
  EXPECT_EQ(testing::AssertionSuccess(), IsLanePositionClose(test_value, close_pos, kTolerance));
  const LanePosition non_close_pos{1.e-2, 10., 10.};
  EXPECT_EQ(testing::AssertionFailure(), IsLanePositionClose(test_value, non_close_pos, kTolerance));
}

TEST(IsRotationCloseTest, Test) {
  const Rotation test_value{Rotation::FromRpy(0., 10., 10.)};
  const Rotation close_rot{Rotation::FromRpy(1.e-4, 10., 10.)};
  EXPECT_EQ(testing::AssertionSuccess(), IsRotationClose(test_value, close_rot, kTolerance));
  const Rotation non_close_rot{Rotation::FromRpy(1.e-2, 10., 10.)};
  EXPECT_EQ(testing::AssertionFailure(), IsRotationClose(test_value, non_close_rot, kTolerance));
}

TEST(IsRBoundsCloseTest, Test) {
  const RBounds test_value{0., 10.};
  const RBounds close_r_bounds{-1.e-4, 10.};
  EXPECT_EQ(testing::AssertionSuccess(), IsRBoundsClose(test_value, close_r_bounds, kTolerance));
  const RBounds close_r_max{-1.e-2, 10.};
  EXPECT_EQ(testing::AssertionFailure(), IsRBoundsClose(test_value, close_r_max, kTolerance));
  const RBounds close_r_min{-1e-4, 1.e-2};
  EXPECT_EQ(testing::AssertionFailure(), IsRBoundsClose(test_value, close_r_min, kTolerance));
  const RBounds non_close_bounds{-1.e-2, 100.};
  EXPECT_EQ(testing::AssertionFailure(), IsRBoundsClose(test_value, non_close_bounds, kTolerance));
}

TEST(IsHBoundsCloseTest, Test) {
  const HBounds test_value{0., 10.};
  const HBounds close_h_bounds{-1.e-4, 10.};
  EXPECT_EQ(testing::AssertionSuccess(), IsHBoundsClose(test_value, close_h_bounds, kTolerance));
  const HBounds close_h_max{-1.e-2, 10.};
  EXPECT_EQ(testing::AssertionFailure(), IsHBoundsClose(test_value, close_h_max, kTolerance));
  const HBounds close_h_min{-1.e-4, 1.e-2};
  EXPECT_EQ(testing::AssertionFailure(), IsHBoundsClose(test_value, close_h_min, kTolerance));
  const HBounds non_close_bounds{-1.e-2, 100.};
  EXPECT_EQ(testing::AssertionFailure(), IsHBoundsClose(test_value, non_close_bounds, kTolerance));
}

TEST(IsLanePositionResultCloseTest, Test) {
  const InertialPosition inertial_pos{0., 10., 10.};
  const LanePosition lane_pos{0., 10., 10.};
  const double distance{10.};
  const LanePositionResult test_value{lane_pos, inertial_pos, distance};
  const LanePositionResult close_value{lane_pos, inertial_pos, distance};
  EXPECT_EQ(testing::AssertionSuccess(), IsLanePositionResultClose(test_value, close_value, kTolerance));

  // Non close per lane position.
  const LanePositionResult non_lane_pos_close{{10., 10., 10.}, inertial_pos, distance};
  EXPECT_EQ(testing::AssertionFailure(), IsLanePositionResultClose(test_value, non_lane_pos_close, kTolerance));

  // Non close per nearest position.
  const LanePositionResult non_inertial_pos_close{lane_pos, {10., 10., 10.}, distance};
  EXPECT_EQ(testing::AssertionFailure(), IsLanePositionResultClose(test_value, non_inertial_pos_close, kTolerance));

  // Non close per distance.
  const LanePositionResult non_distance_close{lane_pos, inertial_pos, 85.};
  EXPECT_EQ(testing::AssertionFailure(), IsLanePositionResultClose(test_value, non_distance_close, kTolerance));
}

TEST(IsRoadPositionResultCloseTest, Test) {
  const InertialPosition inertial_pos{0., 10., 10.};
  const LanePosition lane_pos{0., 10., 10.};
  const RoadPosition road_pos{reinterpret_cast<const Lane*>(0xDeadBeef), lane_pos};
  const double distance{10.};
  const RoadPositionResult test_value{road_pos, inertial_pos, distance};
  const RoadPositionResult close_value{road_pos, inertial_pos, distance};
  EXPECT_EQ(testing::AssertionSuccess(), IsRoadPositionResultClose(test_value, close_value, kTolerance));

  // Non close per non lane matching.
  const RoadPositionResult non_lane_matching{
      {reinterpret_cast<const Lane*>(0xDeadD00d), lane_pos}, inertial_pos, distance};
  EXPECT_EQ(testing::AssertionFailure(), IsRoadPositionResultClose(test_value, non_lane_matching, kTolerance));

  // Non close per lane position.
  const RoadPositionResult non_lane_pos_close{{road_pos.lane, {10., 10., 10.}}, inertial_pos, distance};
  EXPECT_EQ(testing::AssertionFailure(), IsRoadPositionResultClose(test_value, non_lane_pos_close, kTolerance));

  // Non close per nearest position.
  const RoadPositionResult non_inertial_pos_close{road_pos, {10., 10., 10.}, distance};
  EXPECT_EQ(testing::AssertionFailure(), IsRoadPositionResultClose(test_value, non_inertial_pos_close, kTolerance));

  // Non close per distance.
  const RoadPositionResult non_distance_close{road_pos, inertial_pos, 85.};
  EXPECT_EQ(testing::AssertionFailure(), IsRoadPositionResultClose(test_value, non_distance_close, kTolerance));
}

TEST(IsLaneEndEqualTest, Test) {
  const LaneEnd lane_end1(reinterpret_cast<const Lane*>(0xDeadBeef), maliput::api::LaneEnd::Which::kStart);
  const LaneEnd lane_end2(reinterpret_cast<const Lane*>(0xDeadD00d), maliput::api::LaneEnd::Which::kStart);
  const LaneEnd lane_end3(reinterpret_cast<const Lane*>(0xDeadBeef), maliput::api::LaneEnd::Which::kFinish);

  EXPECT_EQ(testing::AssertionSuccess(), IsLaneEndEqual(lane_end1, lane_end1));
  EXPECT_EQ(testing::AssertionFailure(), IsLaneEndEqual(lane_end1, lane_end2));
  EXPECT_EQ(testing::AssertionFailure(), IsLaneEndEqual(lane_end1, lane_end3));
}

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace maliput
