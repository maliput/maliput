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
#include "maliput/routing/route.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/common/assertion_error.h"
#include "maliput/routing/lane_s_range_relation.h"
#include "maliput/routing/route_phase.h"
#include "maliput/routing/route_position_result.h"
#include "routing/road_network_mocks.h"

namespace maliput {
namespace routing {
namespace test {
namespace {

using ::testing::_;
using ::testing::Return;
using ::testing::ReturnRef;

class RouteBaseTest : public ::testing::Test {
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

class RouteConstructorValidationsTest : public RouteBaseTest {
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

  void SetUp() override { RouteBaseTest::SetUp(); }
};

TEST_F(RouteConstructorValidationsTest, EmptyRoutePhasesThrows) {
  EXPECT_THROW({ Route({}, road_network_.get()); }, common::assertion_error);
}

TEST_F(RouteConstructorValidationsTest, RoadNetworkPtrIsNullptrThrows) {
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
  const RoutePhase route_phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                               road_network_.get());

  EXPECT_THROW({ Route({route_phase}, nullptr); }, common::assertion_error);
}

TEST_F(RouteConstructorValidationsTest, CorrectConstruction) {
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
  const RoutePhase route_phase(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                               road_network_.get());

  EXPECT_NO_THROW({ Route({route_phase}, road_network_.get()); });
}

class RouteAccessorsTest : public RouteConstructorValidationsTest {
 public:
  void SetUp() override {
    RouteConstructorValidationsTest::SetUp();

    EXPECT_CALL(lane_a_, do_id()).WillRepeatedly(Return(kLaneIdA));
    EXPECT_CALL(lane_a_, do_length()).WillRepeatedly(Return(kLaneALength));
    EXPECT_CALL(lane_a_, do_to_left()).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(lane_a_, do_to_right()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(lane_b_, do_id()).WillRepeatedly(Return(kLaneIdB));
    EXPECT_CALL(lane_b_, do_length()).WillRepeatedly(Return(kLaneBLength));
    EXPECT_CALL(lane_b_, do_to_left()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(lane_b_, do_to_right()).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositions{
        api::RoadPosition{&lane_a_, api::LanePosition{0., 0., 0.}}};
    const std::vector<api::RoadPosition> kEndRoadPositions{
        api::RoadPosition{&lane_b_, api::LanePosition{kLaneBLength, 0., 0.}}};
    const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

    route_phase_a_ = std::make_unique<RoutePhase>(kIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions,
                                                  kLaneSRanges, road_network_.get());
  }

  LaneMock lane_a_;
  LaneMock lane_b_;
  std::unique_ptr<RoutePhase> route_phase_a_;
};

TEST_F(RouteAccessorsTest, SizeIsOne) {
  const Route dut({*route_phase_a_}, road_network_.get());

  EXPECT_EQ(dut.size(), 1);
}

TEST_F(RouteAccessorsTest, GetReturnsTheRightLaneSRange) {
  const Route dut({*route_phase_a_}, road_network_.get());

  const RoutePhase& route_phase = dut.Get(0);
  EXPECT_EQ(route_phase.index(), route_phase_a_->index());
  EXPECT_EQ(route_phase.lane_s_range_tolerance(), route_phase_a_->lane_s_range_tolerance());
  EXPECT_EQ(route_phase.start_positions().size(), route_phase_a_->start_positions().size());
  EXPECT_EQ(route_phase.start_positions()[0].lane, &lane_a_);
  EXPECT_EQ(route_phase.start_positions()[0].pos.srh(), api::LanePosition(0., 0., 0.).srh());
  EXPECT_EQ(route_phase.end_positions().size(), route_phase_a_->end_positions().size());
  EXPECT_EQ(route_phase.end_positions()[0].lane, &lane_b_);
  EXPECT_EQ(route_phase.end_positions()[0].pos.srh(), api::LanePosition(kLaneBLength, 0., 0.).srh());
  EXPECT_EQ(route_phase.lane_s_ranges().size(), route_phase_a_->lane_s_ranges().size());
  EXPECT_EQ(route_phase.lane_s_ranges()[0].lane_id(), route_phase_a_->lane_s_ranges()[0].lane_id());
  EXPECT_EQ(route_phase.lane_s_ranges()[0].s_range().s0(), route_phase_a_->lane_s_ranges()[0].s_range().s0());
  EXPECT_EQ(route_phase.lane_s_ranges()[0].s_range().s1(), route_phase_a_->lane_s_ranges()[0].s_range().s1());
  EXPECT_EQ(route_phase.lane_s_ranges()[1].lane_id(), route_phase_a_->lane_s_ranges()[1].lane_id());
  EXPECT_EQ(route_phase.lane_s_ranges()[1].s_range().s0(), route_phase_a_->lane_s_ranges()[1].s_range().s0());
  EXPECT_EQ(route_phase.lane_s_ranges()[1].s_range().s1(), route_phase_a_->lane_s_ranges()[1].s_range().s1());
}

TEST_F(RouteAccessorsTest, GetThrowsWhenPassingAWrongIndex) {
  const Route dut({*route_phase_a_}, road_network_.get());

  EXPECT_THROW({ dut.Get(-1); }, std::out_of_range);
  EXPECT_THROW({ dut.Get(2); }, std::out_of_range);
}

TEST_F(RouteAccessorsTest, GetLaneSRangeIndexesWellLaneSRanges) {
  const Route dut({*route_phase_a_}, road_network_.get());

  const api::LaneSRange lane_s_range_a = dut.GetLaneSRange(0, 0);
  const api::LaneSRange lane_s_range_b = dut.GetLaneSRange(0, 1);

  EXPECT_EQ(lane_s_range_a.lane_id(), kLaneIdA);
  EXPECT_EQ(lane_s_range_a.s_range().s0(), 0.);
  EXPECT_EQ(lane_s_range_a.s_range().s1(), kLaneALength);
  EXPECT_EQ(lane_s_range_b.lane_id(), kLaneIdB);
  EXPECT_EQ(lane_s_range_b.s_range().s0(), 0.);
  EXPECT_EQ(lane_s_range_b.s_range().s1(), kLaneBLength);
}

TEST_F(RouteAccessorsTest, GetLaneSRangeThrowsWhenPassingWrongIndex) {
  const Route dut({*route_phase_a_}, road_network_.get());

  EXPECT_THROW({ dut.GetLaneSRange(1, 0); }, std::out_of_range);
  EXPECT_THROW({ dut.GetLaneSRange(-1, 0); }, std::out_of_range);
  EXPECT_THROW({ dut.GetLaneSRange(0, -1); }, std::out_of_range);
  EXPECT_THROW({ dut.GetLaneSRange(0, 2); }, std::out_of_range);
}

TEST_F(RouteAccessorsTest, StartRoutePositionIsOnLaneSRangeA) {
  const Route dut({*route_phase_a_}, road_network_.get());

  const api::RoadPosition& start_road_position = dut.start_route_position();
  EXPECT_EQ(start_road_position.lane, &lane_a_);
  EXPECT_EQ(start_road_position.pos.srh(), api::LanePosition(0., 0., 0.).srh());
}

TEST_F(RouteAccessorsTest, EndRoutePositionIsOnLaneSRangeB) {
  const Route dut({*route_phase_a_}, road_network_.get());

  const api::RoadPosition& end_road_position = dut.end_route_position();
  EXPECT_EQ(end_road_position.lane, &lane_b_);
  EXPECT_EQ(end_road_position.pos.srh(), api::LanePosition(kLaneBLength, 0., 0.).srh());
}

}  // namespace
}  // namespace test
}  // namespace routing
}  // namespace maliput
