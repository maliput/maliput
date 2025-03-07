// BSD 3-Clause License
//
// Copyright (c) 2023, Woven Planet. All rights reserved.
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
#include "maliput/api/branch_point.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"

namespace maliput {
namespace api {
namespace {

using ::testing::_;
using ::testing::Return;
using ::testing::ReturnRef;

// Create a mock class for LaneEndSet.
class MockLaneEndSet : public LaneEndSet {
 public:
  MOCK_CONST_METHOD0(do_size, int());
  MOCK_CONST_METHOD1(do_get, const LaneEnd&(int index));
};

// Create a mock class for BranchPoint.
class MockBranchPoint : public BranchPoint {
 public:
  MOCK_CONST_METHOD0(do_id, BranchPointId());
  MOCK_CONST_METHOD0(do_road_geometry, const RoadGeometry*());
  MOCK_CONST_METHOD1(DoGetConfluentBranches, const LaneEndSet*(const LaneEnd& end));
  MOCK_CONST_METHOD1(DoGetOngoingBranches, const LaneEndSet*(const LaneEnd& end));
  MOCK_CONST_METHOD1(DoGetDefaultBranch, std::optional<LaneEnd>(const LaneEnd& end));
  MOCK_CONST_METHOD0(DoGetASide, const LaneEndSet*());
  MOCK_CONST_METHOD0(DoGetBSide, const LaneEndSet*());
};

// Create a mock class for RoadGeometry.
class MockRoadGeometry : public RoadGeometry {
 public:
  MOCK_CONST_METHOD0(do_id, RoadGeometryId());
  MOCK_CONST_METHOD0(do_num_junctions, int());
  MOCK_CONST_METHOD1(do_junction, const Junction*(int index));
  MOCK_CONST_METHOD0(do_num_branch_points, int());
  MOCK_CONST_METHOD1(do_branch_point, const BranchPoint*(int index));
  MOCK_CONST_METHOD0(DoById, const IdIndex&());
  MOCK_CONST_METHOD2(DoToRoadPosition, RoadPositionResult(const InertialPosition& inertial_position,
                                                          const std::optional<RoadPosition>& hint));
  MOCK_CONST_METHOD2(DoFindRoadPositions,
                     std::vector<RoadPositionResult>(const InertialPosition& inertial_position, double radius));
  MOCK_CONST_METHOD0(do_linear_tolerance, double());
  MOCK_CONST_METHOD0(do_angular_tolerance, double());
  MOCK_CONST_METHOD0(do_scale_length, double());
  MOCK_CONST_METHOD2(DoSampleAheadWaypoints,
                     std::vector<InertialPosition>(const LaneSRoute&, double path_length_sampling_rate));
  MOCK_CONST_METHOD0(do_inertial_to_backend_frame_translation, math::Vector3());
  MOCK_CONST_METHOD1(DoBackendCustomCommand, std::string(const std::string& command));
};

// Test cases for LaneEndSet
TEST(LaneEndSetTest, SizeTest) {
  MockLaneEndSet lane_end_set;
  EXPECT_CALL(lane_end_set, do_size()).WillOnce(Return(3));
  EXPECT_EQ(lane_end_set.size(), 3);
}

TEST(LaneEndSetTest, GetTest) {
  MockLaneEndSet lane_end_set;
  LaneEnd lane_end_1, lane_end_2, lane_end_3;
  EXPECT_CALL(lane_end_set, do_get(0)).WillOnce(ReturnRef(lane_end_1));
  EXPECT_CALL(lane_end_set, do_get(1)).WillOnce(ReturnRef(lane_end_2));
  EXPECT_CALL(lane_end_set, do_get(2)).WillOnce(ReturnRef(lane_end_3));

  EXPECT_EQ(&lane_end_set.get(0), &lane_end_1);
  EXPECT_EQ(&lane_end_set.get(1), &lane_end_2);
  EXPECT_EQ(&lane_end_set.get(2), &lane_end_3);
}

class BranchPointTest : public ::testing::Test {
 public:
  MockBranchPoint dut;
  MockLaneEndSet lane_end_set;
};

TEST_F(BranchPointTest, BranchPointIdTest) {
  const std::string id{"arbitrarily chosen id"};
  BranchPointId branchPointId{id};
  EXPECT_EQ(branchPointId.string(), id);
}

TEST_F(BranchPointTest, IdTest) {
  BranchPointId branchPointId{"arbitrarily chosen id"};
  EXPECT_CALL(dut, do_id()).WillOnce(Return(branchPointId));
  EXPECT_EQ(dut.id(), branchPointId);
}

TEST_F(BranchPointTest, RoadGeometryTest) {
  MockBranchPoint dut;
  const MockRoadGeometry roadGeometry;
  EXPECT_CALL(dut, do_road_geometry()).WillOnce(Return(&roadGeometry));
  EXPECT_EQ(dut.road_geometry(), &roadGeometry);
}

TEST_F(BranchPointTest, GetConfluentBranchesTest) {
  LaneEnd lane_end;

  // Return the mock LaneEndSet when called with any LaneEnd.
  EXPECT_CALL(dut, DoGetConfluentBranches(_)).WillOnce(Return(&lane_end_set));

  const LaneEndSet* result = dut.GetConfluentBranches(lane_end);
  EXPECT_EQ(result, &lane_end_set);
}

TEST_F(BranchPointTest, GetOngoingBranchesTest) {
  LaneEnd lane_end;

  // Return the mock LaneEndSet when called with any LaneEnd.
  EXPECT_CALL(dut, DoGetOngoingBranches(_)).WillOnce(Return(&lane_end_set));

  const LaneEndSet* result = dut.GetOngoingBranches(lane_end);
  EXPECT_EQ(result, &lane_end_set);
}

TEST_F(BranchPointTest, GetDefaultBranchTest) {
  LaneEnd lane_end;
  std::optional<LaneEnd> default_branch;

  // Return the default branch when called with any LaneEnd.
  EXPECT_CALL(dut, DoGetDefaultBranch(_)).WillOnce(Return(default_branch));
  dut.GetDefaultBranch(lane_end);
}

TEST_F(BranchPointTest, GetASideTest) {
  // Return the mock LaneEndSet for the A-side.
  EXPECT_CALL(dut, DoGetASide()).WillOnce(Return(&lane_end_set));

  const LaneEndSet* result = dut.GetASide();
  EXPECT_EQ(result, &lane_end_set);
}

TEST_F(BranchPointTest, GetBSideTest) {
  // Return the mock LaneEndSet for the B-side.
  EXPECT_CALL(dut, DoGetBSide()).WillOnce(Return(&lane_end_set));

  const LaneEndSet* result = dut.GetBSide();
  EXPECT_EQ(result, &lane_end_set);
}

}  // namespace
}  // namespace api
}  // namespace maliput
