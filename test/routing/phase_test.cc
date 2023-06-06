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
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/common/assertion_error.h"
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
  static constexpr int kWrongIndex{-2};
  static constexpr double kLaneSRangeTolerance{1e-3};
  static constexpr double kWrongLaneSRangeTolerance{-1e-3};
  static constexpr double kLaneALength{100.};
  static constexpr double kLaneBLength{100.};
  static constexpr api::RoadNetwork* kNullptrRoadNetwork{nullptr};

  const api::LaneId kLaneIdA{"lane_a"};
  const api::LaneId kLaneIdB{"lane_b"};
  const api::LaneSRange kLaneSRangeA{kLaneIdA, api::SRange{0., kLaneALength}};
  const api::LaneSRange kLaneSRangeB{kLaneIdB, api::SRange{0., kLaneBLength}};
  const api::LaneSRange kLaneSRangeAShort{kLaneIdA, api::SRange{10., kLaneALength - 10.}};
  const std::vector<api::RoadPosition> kEmptyRoadPositions{};

  void SetUp() override { PhaseBaseTest::SetUp(); }
};

TEST_F(PhaseConstructorValidationsTest, NegativeIndexThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kWrongIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, NegativeLaneSRangeToleranceThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kWrongLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EmptyStartPositionsThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kEmptyRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EmptyEndPositionsThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEmptyRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, InvalidRoadPositionsAtStartPositionsThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kInvalidStartRoadPositions{
      api::RoadPosition{nullptr, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kInvalidStartRoadPositions, kEndRoadPositions, kLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, InvalidRoadPositionsAtEndPositionsThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
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
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  EXPECT_CALL(lane_b, do_length()).WillRepeatedly(Return(kLaneBLength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_b, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EndRoadPositionsNotInRoadNetworkThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  EXPECT_CALL(lane_b, do_length()).WillRepeatedly(Return(kLaneBLength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_b, api::LanePosition{kLaneBLength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, StartRoadPositionsNotInLaneSRangesThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneSRangeAShort.s_range().s1(), 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeAShort};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EndRoadPositionsNotInLaneSRangesThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneSRangeAShort.s_range().s0(), 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeAShort};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, NullptrRoadNetworkThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, kNullptrRoadNetwork);
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, EmptyLaneSRangesThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kEmptyLaneSRanges{};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kEmptyLaneSRanges,
              road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, LaneSRangesNotInRoadNetworkThrows) {
  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  EXPECT_CALL(lane_b, do_length()).WillRepeatedly(Return(kLaneBLength));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, LaneSRangesNotAdjacentThrows) {
  LaneMock lane_a;
  LaneMock lane_b;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(lane_a, do_to_left()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_a, do_to_right()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  EXPECT_CALL(lane_b, do_length()).WillRepeatedly(Return(kLaneBLength));
  EXPECT_CALL(lane_b, do_to_left()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_b, do_to_right()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

  EXPECT_THROW(
      {
        Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
      },
      common::assertion_error);
}

TEST_F(PhaseConstructorValidationsTest, CorrectConstruction) {
  LaneMock lane_a;
  LaneMock lane_b;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(lane_a, do_to_left()).WillRepeatedly(Return(&lane_b));
  EXPECT_CALL(lane_a, do_to_right()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  EXPECT_CALL(lane_b, do_length()).WillRepeatedly(Return(kLaneBLength));
  EXPECT_CALL(lane_b, do_to_left()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_b, do_to_right()).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}},
      api::RoadPosition{&lane_b, api::LanePosition{kLaneBLength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

  EXPECT_NO_THROW({
    Phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges, road_network_.get());
  });
}

class PhaseAccessorsTest : public PhaseConstructorValidationsTest {};

TEST_F(PhaseAccessorsTest, CorrectConstruction) {
  LaneMock lane_a;
  LaneMock lane_b;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(lane_a, do_to_left()).WillRepeatedly(Return(&lane_b));
  EXPECT_CALL(lane_a, do_to_right()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  EXPECT_CALL(lane_b, do_length()).WillRepeatedly(Return(kLaneBLength));
  EXPECT_CALL(lane_b, do_to_left()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_b, do_to_right()).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}},
      api::RoadPosition{&lane_b, api::LanePosition{kLaneBLength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());

  EXPECT_EQ(kIndex, dut.index());
  EXPECT_EQ(kStartRoadPositions.size(), dut.start_positions().size());
  EXPECT_EQ(kStartRoadPositions[0].lane, dut.start_positions()[0].lane);
  EXPECT_EQ(kStartRoadPositions[0].pos.s(), dut.start_positions()[0].pos.s());
  EXPECT_EQ(kStartRoadPositions[0].pos.r(), dut.start_positions()[0].pos.r());
  EXPECT_EQ(kStartRoadPositions[0].pos.h(), dut.start_positions()[0].pos.h());
  EXPECT_EQ(kEndRoadPositions.size(), dut.end_positions().size());
  EXPECT_EQ(kEndRoadPositions[0].lane, dut.end_positions()[0].lane);
  EXPECT_EQ(kEndRoadPositions[0].pos.s(), dut.end_positions()[0].pos.s());
  EXPECT_EQ(kEndRoadPositions[0].pos.r(), dut.end_positions()[0].pos.r());
  EXPECT_EQ(kEndRoadPositions[0].pos.h(), dut.end_positions()[0].pos.h());
  EXPECT_EQ(kEndRoadPositions[1].lane, dut.end_positions()[1].lane);
  EXPECT_EQ(kEndRoadPositions[1].pos.s(), dut.end_positions()[1].pos.s());
  EXPECT_EQ(kEndRoadPositions[1].pos.r(), dut.end_positions()[1].pos.r());
  EXPECT_EQ(kEndRoadPositions[1].pos.h(), dut.end_positions()[1].pos.h());
  EXPECT_EQ(kLaneSRanges.size(), dut.lane_s_ranges().size());
  EXPECT_EQ(kLaneSRanges[0].lane_id(), dut.lane_s_ranges()[0].lane_id());
  EXPECT_EQ(kLaneSRanges[0].s_range().s0(), dut.lane_s_ranges()[0].s_range().s0());
  EXPECT_EQ(kLaneSRanges[0].s_range().s1(), dut.lane_s_ranges()[0].s_range().s1());
  EXPECT_EQ(kLaneSRanges[1].lane_id(), dut.lane_s_ranges()[1].lane_id());
  EXPECT_EQ(kLaneSRanges[1].s_range().s0(), dut.lane_s_ranges()[1].s_range().s0());
  EXPECT_EQ(kLaneSRanges[1].s_range().s1(), dut.lane_s_ranges()[1].s_range().s1());
}

class PhaseMappingTest : public PhaseConstructorValidationsTest {};

TEST_F(PhaseAccessorsTest, FindPhasePositionByInertialPositionWithOneElement) {
  const api::LanePositionResult kLanePositionResult{api::LanePosition{10., 0., 0.}, api::InertialPosition{1., 0., 0.},
                                                    0.2};
  const api::InertialPosition kInertialPosition{1., 2., 0.};

  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(lane_a, do_to_left()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_a, do_to_right()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_a, DoToLanePosition(_)).WillRepeatedly(Return(kLanePositionResult));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};
  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());

  const PhasePositionResult position_result = dut.FindPhasePositionBy(kInertialPosition);

  EXPECT_EQ(0, position_result.lane_s_range_index);
  EXPECT_EQ(kLanePositionResult.lane_position.s(), position_result.lane_position.s());
  EXPECT_EQ(kLanePositionResult.lane_position.r(), position_result.lane_position.r());
  EXPECT_EQ(kLanePositionResult.lane_position.h(), position_result.lane_position.h());
  EXPECT_EQ(kLanePositionResult.nearest_position.x(), position_result.inertial_position.x());
  EXPECT_EQ(kLanePositionResult.nearest_position.y(), position_result.inertial_position.y());
  EXPECT_EQ(kLanePositionResult.nearest_position.z(), position_result.inertial_position.z());
  EXPECT_EQ(kLanePositionResult.distance, position_result.distance);
}

TEST_F(PhaseAccessorsTest, FindPhasePositionByRoadPositionOutsideLaneSRange) {
  const api::LanePositionResult kLanePositionResult{api::LanePosition{10., 0., 0.}, api::InertialPosition{1., 0., 0.},
                                                    0.2};
  const api::InertialPosition kInertialPosition{1., 2., 0.};

  LaneMock lane_a;
  EXPECT_CALL(lane_a, do_id()).WillRepeatedly(Return(kLaneIdA));
  EXPECT_CALL(lane_a, do_length()).WillRepeatedly(Return(kLaneALength));
  EXPECT_CALL(lane_a, do_to_left()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_a, do_to_right()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_a, DoToLanePosition(_)).WillRepeatedly(Return(kLanePositionResult));
  LaneMock lane_b;
  EXPECT_CALL(lane_b, do_id()).WillRepeatedly(Return(kLaneIdB));
  EXPECT_CALL(lane_b, do_length()).WillRepeatedly(Return(kLaneBLength));
  EXPECT_CALL(lane_b, do_to_left()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_b, do_to_right()).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(lane_b, DoToInertialPosition(_)).WillRepeatedly(Return(kInertialPosition));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a));
  EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b));
  EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

  const api::RoadPosition kRoadPosition{&lane_b, api::LanePosition{10., 0., 0.}};

  const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a, api::LanePosition{0., 0., 0.}}};
  const std::vector<api::RoadPosition> kEndRoadPositions{
      api::RoadPosition{&lane_a, api::LanePosition{kLaneALength, 0., 0.}}};
  const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA};
  const Phase dut(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                  road_network_.get());

  const PhasePositionResult position_result = dut.FindPhasePositionBy(kInertialPosition);

  EXPECT_EQ(0, position_result.lane_s_range_index);
  EXPECT_EQ(kLanePositionResult.lane_position.s(), position_result.lane_position.s());
  EXPECT_EQ(kLanePositionResult.lane_position.r(), position_result.lane_position.r());
  EXPECT_EQ(kLanePositionResult.lane_position.h(), position_result.lane_position.h());
  EXPECT_EQ(kLanePositionResult.nearest_position.x(), position_result.inertial_position.x());
  EXPECT_EQ(kLanePositionResult.nearest_position.y(), position_result.inertial_position.y());
  EXPECT_EQ(kLanePositionResult.nearest_position.z(), position_result.inertial_position.z());
  EXPECT_EQ(kLanePositionResult.distance, position_result.distance);
}

}  // namespace
}  // namespace test
}  // namespace routing
}  // namespace maliput
