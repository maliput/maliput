// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota. All rights reserved.
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
#include "maliput/routing/phase.h"

#include <memory>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/maliput_routing_position_compare.h"
#include "maliput/test_utilities/maliput_types_compare.h"
#include "maliput/test_utilities/regions_test_utilities.h"
#include "routing/road_network_mocks.h"

namespace maliput {
namespace routing {
namespace test {
namespace {

using ::testing::_;
using ::testing::Return;
using ::testing::ReturnRef;

class PhaseBaseTest : public ::testing::Test {
 public:
  void SetUp() override {
    road_network_ = MakeMockedRoadNetwork();
    road_geometry_ptr_ =
        const_cast<RoadGeometryMock*>(static_cast<const RoadGeometryMock*>(road_network_->road_geometry()));
  }

  std::unique_ptr<api::RoadNetwork> road_network_;
  RoadGeometryMock* road_geometry_ptr_{};
  IdIndexMock id_index_;
};

class PhaseConstructorValidationsTest : public PhaseBaseTest {
 public:
  static constexpr int kIndex{1};
  static constexpr double kLaneSRangeTolerance{1e-3};
  static constexpr double kLaneALength{100.};
  static constexpr double kLaneBLength{100.};
  static constexpr api::RoadNetwork* kNullptrRoadNetwork{nullptr};

  const api::LaneId kLaneIdA{"lane_a"};
  const api::LaneId kLaneIdB{"lane_b"};
  const api::LaneSRange kLaneSRangeA{kLaneIdA, api::SRange{0., kLaneALength}};
  const api::LaneSRange kLaneSRangeB{kLaneIdB, api::SRange{0., kLaneBLength}};
  const api::LaneSRange kLaneSRangeAShort{kLaneIdA, api::SRange{10., kLaneALength - 10.}};
  const std::vector<api::RoadPosition> kEmptyRoadPositions{};

  void SetUp() override {
    PhaseBaseTest::SetUp();
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

    EXPECT_CALL(lane_a_, do_id()).WillRepeatedly(Return(kLaneIdA));
    EXPECT_CALL(lane_a_, do_length()).WillRepeatedly(Return(kLaneALength));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
  }

  void ConfigureLaneBMockOutsideRoadGeometry() {
    EXPECT_CALL(lane_b_, do_id()).WillRepeatedly(Return(kLaneIdB));
    EXPECT_CALL(lane_b_, do_length()).WillRepeatedly(Return(kLaneBLength));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(nullptr));
  }

  void ConfigureLaneBMockInsideRoadGeometry() {
    EXPECT_CALL(lane_b_, do_id()).WillRepeatedly(Return(kLaneIdB));
    EXPECT_CALL(lane_b_, do_length()).WillRepeatedly(Return(kLaneBLength));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
  }

  void ConfigureAdjacentLanes(LaneMock* lane, LaneMock* left_lane = nullptr, LaneMock* right_lane = nullptr) {
    EXPECT_CALL(*lane, do_to_left()).WillRepeatedly(Return(left_lane));
    EXPECT_CALL(*lane, do_to_right()).WillRepeatedly(Return(right_lane));
  }

  LaneMock lane_a_;
  LaneMock lane_b_;
};

TEST_F(PhaseConstructorValidationsTest, NegativeIndexThrows) {
  static constexpr int kWrongIndex{-2};
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kWrongIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, NegativeLaneSRangeToleranceThrows) {
  static constexpr double kWrongLaneSRangeTolerance{-1e-3};
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kWrongLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EmptyStartPositionsThrows) {
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kEmptyRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EmptyEndPositionsThrows) {
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEmptyRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, InvalidRoadPositionsAtStartPositionsThrows) {
  const std::vector<api::RoadPosition> kInvalidStartRoadPositions{
      api::RoadPosition{nullptr, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kInvalidStartRoadPositions, kEndRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, InvalidRoadPositionsAtEndPositionsThrows) {
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kInvalidEndRoadPositions{
      api::RoadPosition{nullptr, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kInvalidEndRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, StartRoadPositionsNotInRoadNetworkThrows) {
  ConfigureLaneBMockOutsideRoadGeometry();
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_b_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EndRoadPositionsNotInRoadNetworkThrows) {
  ConfigureLaneBMockOutsideRoadGeometry();
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_b_, api::LanePosition{kLaneBLength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, StartRoadPositionsNotInLaneSRangesThrows) {
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneSRangeAShort.s_range().s1(), 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeAShort};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EndRoadPositionsNotInLaneSRangesThrows) {
  const std::vector<api::RoadPosition> kStartRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneSRangeAShort.s_range().s0(), 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeAShort};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, NullptrRoadNetworkThrows) {
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, kNullptrRoadNetwork);
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EmptyLaneSRangesThrows) {
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kEmptyLaneSRanges{};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kEmptyLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, LaneSRangesNotInRoadNetworkThrows) {
  ConfigureLaneBMockOutsideRoadGeometry();
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, LaneSRangesNotAdjacentThrows) {
  ConfigureAdjacentLanes(&lane_a_);
  ConfigureLaneBMockInsideRoadGeometry();
  ConfigureAdjacentLanes(&lane_b_);
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, CorrectConstruction) {
  ConfigureAdjacentLanes(&lane_a_, &lane_b_ /* left lane */, nullptr /* right lane*/);
  ConfigureLaneBMockInsideRoadGeometry();
  ConfigureAdjacentLanes(&lane_b_, nullptr /* left lane */, &lane_a_ /* right lane*/);
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}},
      api::RoadPosition{&lane_b_, api::LanePosition{kLaneBLength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

  EXPECT_NO_THROW({
    Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
  });
}

class PhaseAccessorsTest : public PhaseConstructorValidationsTest {};

TEST_F(PhaseAccessorsTest, CorrectConstruction) {
  ConfigureAdjacentLanes(&lane_a_, &lane_b_ /* left lane */, nullptr /* right lane*/);
  ConfigureLaneBMockInsideRoadGeometry();
  ConfigureAdjacentLanes(&lane_b_, nullptr /* left lane */, &lane_a_ /* right lane*/);
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}},
      api::RoadPosition{&lane_b_, api::LanePosition{kLaneBLength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());

  EXPECT_EQ(kIndex, dut.index());
  EXPECT_EQ(kLaneSRangeTolerance, dut.lane_s_range_tolerance());
  EXPECT_EQ(kStartRoadPositions.size(), dut.start_positions().size());
  EXPECT_EQ(kStartRoadPositions[0].lane, dut.start_positions()[0].lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(kStartRoadPositions[0].pos, dut.start_positions()[0].pos, 0.));
  EXPECT_EQ(kEndRoadPositions.size(), dut.end_positions().size());
  EXPECT_EQ(kEndRoadPositions[0].lane, dut.end_positions()[0].lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(kEndRoadPositions[0].pos, dut.end_positions()[0].pos, 0.));
  EXPECT_EQ(kEndRoadPositions[1].lane, dut.end_positions()[1].lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(kEndRoadPositions[1].pos, dut.end_positions()[1].pos, 0.));
  EXPECT_EQ(kLaneSRanges.size(), dut.lane_s_ranges().size());
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(kLaneSRanges[0], dut.lane_s_ranges()[0]));
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(kLaneSRanges[1], dut.lane_s_ranges()[1]));
}

class PhaseMappingTest : public PhaseConstructorValidationsTest {};

TEST_F(PhaseAccessorsTest, FindPhasePositionByInertialPositionWithSingleLaneSRangePhase) {
  const api::LanePositionResult kLanePositionResult{api::LanePosition{10., 0., 0.}, api::InertialPosition{1., 0., 0.},
                                                    0.2};
  const api::InertialPosition kInertialPosition{1., 2., 0.};
  ConfigureAdjacentLanes(&lane_a_);
  EXPECT_CALL(lane_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLanePositionResult));
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};
  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());
  const PhasePositionResult kExpectedPhasePositionResult{
      0, kLanePositionResult.lane_position, kLanePositionResult.nearest_position, kLanePositionResult.distance};

  const PhasePositionResult position_result = dut.FindPhasePosition(kInertialPosition);

  EXPECT_TRUE(IsPhasePositionResultClose(kExpectedPhasePositionResult, position_result, 0.));
}

TEST_F(PhaseAccessorsTest, FindPhasePositionByInertialPositionWithMultipleLaneSRangePhase) {
  ConfigureAdjacentLanes(&lane_a_, &lane_b_ /* left lane */, nullptr /* right lane*/);
  ConfigureLaneBMockInsideRoadGeometry();
  ConfigureAdjacentLanes(&lane_b_, nullptr /* left lane */, &lane_a_ /* right lane*/);
  SegmentMock segment;
  EXPECT_CALL(segment, do_id()).WillRepeatedly(Return(api::SegmentId("segment_mock")));
  EXPECT_CALL(lane_a_, do_segment()).WillRepeatedly(Return(&segment));
  EXPECT_CALL(lane_b_, do_segment()).WillRepeatedly(Return(&segment));
  const api::RBounds lane_lane_bounds(-2.5, 2.5);
  EXPECT_CALL(lane_a_, do_lane_bounds(_)).WillRepeatedly(Return(lane_lane_bounds));
  EXPECT_CALL(lane_b_, do_lane_bounds(_)).WillRepeatedly(Return(lane_lane_bounds));
  const api::LanePositionResult kLaneAPositionResult{api::LanePosition{10., 0., 0.}, api::InertialPosition{1., 0., 0.},
                                                     0.};
  EXPECT_CALL(lane_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneAPositionResult));
  const api::LanePositionResult kLaneBPositionResult{api::LanePosition{10., 5., 0.}, api::InertialPosition{1., 0., 0.},
                                                     0.};
  EXPECT_CALL(lane_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneBPositionResult));
  const api::InertialPosition kInertialPosition{1., 2., 0.};
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};
  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());
  const PhasePositionResult kExpectedPhasePositionResult{
      0, kLaneAPositionResult.lane_position, kLaneAPositionResult.nearest_position, kLaneAPositionResult.distance};

  const PhasePositionResult position_result = dut.FindPhasePosition(kInertialPosition);

  EXPECT_TRUE(IsPhasePositionResultClose(kExpectedPhasePositionResult, position_result, 0.));
}

TEST_F(PhaseAccessorsTest, FindPhasePositionByInertialPositionOutsidePhase) {
  const api::LanePositionResult kLanePositionResult{api::LanePosition{5., 0., 0.}, api::InertialPosition{1., 0., 0.},
                                                    0.2};
  const api::InertialPosition kInertialPosition{1., 2., 0.};
  const api::InertialPosition kNearestInertialPosition{3., 4., 0.};
  const double distance = kNearestInertialPosition.Distance(kInertialPosition);
  ConfigureAdjacentLanes(&lane_a_);
  EXPECT_CALL(lane_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLanePositionResult));
  EXPECT_CALL(lane_a_, DoToInertialPosition(_)).WillRepeatedly(Return(kNearestInertialPosition));
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{10., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength - 10., 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeAShort};
  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());
  const PhasePositionResult kExpectedPhasePositionResult{0, api::LanePosition{kLaneSRangeAShort.s_range().s0(), 0., 0.},
                                                         kNearestInertialPosition, distance};

  const PhasePositionResult position_result = dut.FindPhasePosition(kInertialPosition);

  EXPECT_TRUE(IsPhasePositionResultClose(kExpectedPhasePositionResult, position_result, 0.));
}

TEST_F(PhaseAccessorsTest, FindPhasePositionByRoadPositionThrowsWithInvalidPosition) {
  ConfigureAdjacentLanes(&lane_a_);
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};
  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());
  const api::RoadPosition kInvalidRoadPosition{nullptr, api::LanePosition{10., 0., 0.}};

  EXPECT_THROW({ dut.FindPhasePosition(kInvalidRoadPosition); }, common::assertion_error);
}

TEST_F(PhaseAccessorsTest, FindPhasePositionByRoadPositionOutsidePhase) {
  const api::LanePositionResult kLanePositionResult{api::LanePosition{10., 0., 0.}, api::InertialPosition{1., 0., 0.},
                                                    0.2};
  const api::InertialPosition kInertialPosition{1., 2., 0.};
  ConfigureAdjacentLanes(&lane_a_);
  EXPECT_CALL(lane_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLanePositionResult));
  ConfigureLaneBMockInsideRoadGeometry();
  ConfigureAdjacentLanes(&lane_b_);
  EXPECT_CALL(lane_b_, DoToInertialPosition(_)).WillRepeatedly(Return(kInertialPosition));
  const api::RoadPosition kRoadPosition{&lane_b_, api::LanePosition{10., 0., 0.}};
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};
  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());
  const PhasePositionResult kExpectedPhasePositionResult{
      0, kLanePositionResult.lane_position, kLanePositionResult.nearest_position, kLanePositionResult.distance};

  const PhasePositionResult position_result = dut.FindPhasePosition(kInertialPosition);

  EXPECT_TRUE(IsPhasePositionResultClose(kExpectedPhasePositionResult, position_result, 0.));
}

TEST_F(PhaseAccessorsTest, FindPhasePositionByRoadPositionInsidePhase) {
  ConfigureAdjacentLanes(&lane_a_, &lane_b_ /* left lane */, nullptr /* right lane*/);
  ConfigureLaneBMockInsideRoadGeometry();
  ConfigureAdjacentLanes(&lane_b_, nullptr /* left lane */, &lane_a_ /* right lane*/);
  const api::InertialPosition kInertialPosition{1., 2., 0.};
  EXPECT_CALL(lane_b_, DoToInertialPosition(_)).WillRepeatedly(Return(kInertialPosition));
  const api::LanePositionResult kLanePositionResult{api::LanePosition{10., 0., 0.}, api::InertialPosition{1., 0., 0.},
                                                    0.2};
  EXPECT_CALL(lane_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLanePositionResult));
  const api::RoadPosition kRoadPosition{&lane_b_, api::LanePosition{10., 0., 0.}};
  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a_, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};
  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());
  const PhasePositionResult kExpectedPhasePositionResult{
      1, kLanePositionResult.lane_position, kLanePositionResult.nearest_position, kLanePositionResult.distance};

  const PhasePositionResult position_result = dut.FindPhasePosition(kRoadPosition);

  EXPECT_TRUE(IsPhasePositionResultClose(kExpectedPhasePositionResult, position_result, 0.));
}

// TODO: Test Phase::FindPhasePosition() when the LaneSRange::WithS() is false.

class ValidatePositionIsInLaneSRangesTest : public ::testing::Test {
 public:
  static constexpr double kLaneSRangeTolerance{1e-3};
  const api::LaneId kLaneIdA{"lane_a"};
  const api::LaneId kLaneIdB{"lane_b"};
  const api::LaneId kLaneIdC{"lane_c"};
  const api::LaneSRange kLaneSRangeA{kLaneIdA, api::SRange{10., 100.}};
  const api::LaneSRange kLaneSRangeB{kLaneIdB, api::SRange{5., 105}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};
  const api::LanePosition kLanePositionInsideRangeA{50., 0., 0.};
  const api::LanePosition kLanePositionOutsideRangeB{3., 0., 0.};
  const api::LanePosition kLanePositionWithinToleranceRangeB{4.9995, 0., 0.};

  void SetUp() override {
    EXPECT_CALL(lane_a_, do_id()).WillRepeatedly(Return(kLaneIdA));
    EXPECT_CALL(lane_b_, do_id()).WillRepeatedly(Return(kLaneIdB));
    EXPECT_CALL(lane_c_, do_id()).WillRepeatedly(Return(kLaneIdC));
  }

  LaneMock lane_a_;
  LaneMock lane_b_;
  LaneMock lane_c_;
};

TEST_F(ValidatePositionIsInLaneSRangesTest, InvalidRoadPositionThrows) {
  const api::RoadPosition position;
  EXPECT_THROW({ ValidatePositionIsInLaneSRanges(position, kLaneSRanges, kLaneSRangeTolerance); },
               common::assertion_error);
}

TEST_F(ValidatePositionIsInLaneSRangesTest, EmptyLaneSRangeThrows) {
  const api::RoadPosition position(&lane_a_, kLanePositionInsideRangeA);
  EXPECT_THROW({ ValidatePositionIsInLaneSRanges(position, {}, kLaneSRangeTolerance); }, common::assertion_error);
}

TEST_F(ValidatePositionIsInLaneSRangesTest, NegativeToleranceThrows) {
  const api::RoadPosition position(&lane_a_, kLanePositionInsideRangeA);
  EXPECT_THROW({ ValidatePositionIsInLaneSRanges(position, kLaneSRanges, -1.); }, common::assertion_error);
}

TEST_F(ValidatePositionIsInLaneSRangesTest, PositionWithinRangeAReturnsTrue) {
  const api::RoadPosition position(&lane_a_, kLanePositionInsideRangeA);
  EXPECT_TRUE(ValidatePositionIsInLaneSRanges(position, kLaneSRanges, kLaneSRangeTolerance));
}

TEST_F(ValidatePositionIsInLaneSRangesTest, PositionOutsideRangeBReturnsFalse) {
  const api::RoadPosition position(&lane_b_, kLanePositionOutsideRangeB);
  EXPECT_FALSE(ValidatePositionIsInLaneSRanges(position, kLaneSRanges, kLaneSRangeTolerance));
}

TEST_F(ValidatePositionIsInLaneSRangesTest, PositionWithinToleranceRangeBReturnsTrue) {
  const api::RoadPosition position(&lane_b_, kLanePositionWithinToleranceRangeB);
  EXPECT_TRUE(ValidatePositionIsInLaneSRanges(position, kLaneSRanges, kLaneSRangeTolerance));
}

TEST_F(ValidatePositionIsInLaneSRangesTest, PositionWithNoMatchingIdReturnsFalse) {
  const api::RoadPosition position(&lane_c_, kLanePositionInsideRangeA);
  EXPECT_FALSE(ValidatePositionIsInLaneSRanges(position, kLaneSRanges, kLaneSRangeTolerance));
}

}  // namespace
}  // namespace test
}  // namespace routing
}  // namespace maliput
