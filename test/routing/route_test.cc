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

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/common/assertion_error.h"
#include "maliput/routing/lane_s_range_relation.h"
#include "maliput/routing/phase.h"
#include "maliput/routing/route_position_result.h"
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
  static constexpr int kPhaseIndex{1};
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

TEST_F(RouteConstructorValidationsTest, EmptyPhasesThrows) {
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
  const Phase phase(kPhaseIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                    road_network_.get());

  EXPECT_THROW({ Route({phase}, nullptr); }, common::assertion_error);
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
  const Phase phase(kPhaseIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions, kLaneSRanges,
                    road_network_.get());

  EXPECT_NO_THROW({ Route({phase}, road_network_.get()); });
}

class RouteAccessorsTest : public RouteConstructorValidationsTest {
 public:
  static constexpr int kPhaseAIndex{0};
  const api::LanePosition kStartRouteLanePosition{0., 0., 0.};
  const api::LanePosition kEndRouteLanePosition{kLaneBLength, 0., 0.};

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
    const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, kStartRouteLanePosition}};
    const std::vector<api::RoadPosition> kEndRoadPositions{api::RoadPosition{&lane_b_, kEndRouteLanePosition}};
    const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

    phase_a_ = std::make_unique<Phase>(kPhaseAIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions,
                                       kLaneSRanges, road_network_.get());
  }

  LaneMock lane_a_;
  LaneMock lane_b_;
  std::unique_ptr<Phase> phase_a_;
};

TEST_F(RouteAccessorsTest, SizeIsOne) {
  const Route dut({*phase_a_}, road_network_.get());

  EXPECT_EQ(dut.size(), 1);
}

TEST_F(RouteAccessorsTest, GetReturnsTheRightLaneSRange) {
  const Route dut({*phase_a_}, road_network_.get());

  const Phase& phase = dut.Get(0);

  EXPECT_EQ(phase.index(), phase_a_->index());
  EXPECT_EQ(phase.lane_s_range_tolerance(), phase_a_->lane_s_range_tolerance());
  EXPECT_EQ(phase.start_positions().size(), phase_a_->start_positions().size());
  EXPECT_EQ(phase.start_positions()[0].lane, &lane_a_);
  EXPECT_TRUE(api::test::IsLanePositionClose(kStartRouteLanePosition, phase.start_positions()[0].pos, 0.));
  EXPECT_EQ(phase.end_positions().size(), phase_a_->end_positions().size());
  EXPECT_EQ(phase.end_positions()[0].lane, &lane_b_);
  EXPECT_TRUE(api::test::IsLanePositionClose(kEndRouteLanePosition, phase.end_positions()[0].pos, 0.));
  EXPECT_EQ(phase.lane_s_ranges().size(), phase_a_->lane_s_ranges().size());
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(phase_a_->lane_s_ranges()[0], phase.lane_s_ranges()[0]));
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(phase_a_->lane_s_ranges()[1], phase.lane_s_ranges()[1]));
}

TEST_F(RouteAccessorsTest, GetThrowsWhenPassingAWrongIndex) {
  const Route dut({*phase_a_}, road_network_.get());

  EXPECT_THROW({ dut.Get(-1); }, std::out_of_range);
  EXPECT_THROW({ dut.Get(2); }, std::out_of_range);
}

TEST_F(RouteAccessorsTest, StartRoutePositionIsOnLaneSRangeA) {
  const Route dut({*phase_a_}, road_network_.get());

  const api::RoadPosition& start_road_position = dut.start_route_position();

  EXPECT_EQ(start_road_position.lane, &lane_a_);
  EXPECT_TRUE(api::test::IsLanePositionClose(kStartRouteLanePosition, start_road_position.pos, 0.));
}

TEST_F(RouteAccessorsTest, EndRoutePositionIsOnLaneSRangeB) {
  const Route dut({*phase_a_}, road_network_.get());

  const api::RoadPosition& end_road_position = dut.end_route_position();

  EXPECT_EQ(end_road_position.lane, &lane_b_);
  EXPECT_TRUE(api::test::IsLanePositionClose(kEndRouteLanePosition, end_road_position.pos, 0.));
}

class RouteWithOnePhaseTest : public RouteAccessorsTest {
 public:
  const api::SegmentId kSegmentAId{"segment_a"};

  void SetUp() override {
    RouteAccessorsTest::SetUp();
    EXPECT_CALL(segment_a_, do_id()).WillRepeatedly(Return(kSegmentAId));
    EXPECT_CALL(lane_a_, do_segment()).WillRepeatedly(Return(&segment_a_));
    EXPECT_CALL(lane_b_, do_segment()).WillRepeatedly(Return(&segment_a_));
    const api::RBounds lane_lane_bounds(-2.5, 2.5);
    EXPECT_CALL(lane_a_, do_lane_bounds(_)).WillRepeatedly(Return(lane_lane_bounds));
    EXPECT_CALL(lane_b_, do_lane_bounds(_)).WillRepeatedly(Return(lane_lane_bounds));
  }

  SegmentMock segment_a_;
};

TEST_F(RouteWithOnePhaseTest, FindRoutePositionByInertialPositionIsMappedViaPhase) {
  const api::LanePositionResult kLaneAPositionResult{api::LanePosition{10., 5., 0.}, api::InertialPosition{1., 0., 0.},
                                                     0.};
  EXPECT_CALL(lane_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneAPositionResult));
  const api::LanePositionResult kLaneBPositionResult{api::LanePosition{10., 0., 0.}, api::InertialPosition{1., 0., 0.},
                                                     0.};
  EXPECT_CALL(lane_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneBPositionResult));
  const api::InertialPosition kInertialPosition{1., 2., 0.};
  const Route dut({*phase_a_}, road_network_.get());
  const RoutePositionResult kExpectedRoutePositionResult{
      kPhaseAIndex, PhasePositionResult{1, kLaneBPositionResult.lane_position, kLaneBPositionResult.nearest_position,
                                        kLaneBPositionResult.distance}};

  const RoutePositionResult position_result = dut.FindRoutePosition(kInertialPosition);

  EXPECT_TRUE(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.));
}

TEST_F(RouteWithOnePhaseTest, FindRoutePositionByRoadPositionThrowsWhenInvalid) {
  const Route dut({*phase_a_}, road_network_.get());

  EXPECT_THROW({ dut.FindRoutePosition(api::RoadPosition{}); }, common::assertion_error);
}

// Failed -- to be checked.
TEST_F(RouteWithOnePhaseTest, FindRoutePositionByRoadPositionWhenRoadPositionIsNotInRouteBecomesByInertialPosition) {
  const api::LanePositionResult kLaneAPositionResult{api::LanePosition{10., 5., 0.}, api::InertialPosition{1., 0., 0.},
                                                     2.};
  EXPECT_CALL(lane_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneAPositionResult));
  const api::LanePositionResult kLaneBPositionResult{api::LanePosition{10., 2.5, 0.}, api::InertialPosition{1., 0., 0.},
                                                     2.};
  EXPECT_CALL(lane_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneBPositionResult));
  const api::InertialPosition kInertialPosition{1., 2., 0.};
  LaneMock lane_c;
  EXPECT_CALL(lane_c, do_id()).WillRepeatedly(Return(api::LaneId("lane_c")));
  EXPECT_CALL(lane_c, DoToInertialPosition(_)).WillRepeatedly(Return(kInertialPosition));
  const api::RoadPosition road_position{&lane_c, api::LanePosition{0., 0., 0.}};
  const Route dut({*phase_a_}, road_network_.get());
  const RoutePositionResult kExpectedRoutePositionResult{
      kPhaseAIndex, PhasePositionResult{1, kLaneBPositionResult.lane_position, kLaneBPositionResult.nearest_position,
                                        kLaneBPositionResult.distance}};

  const RoutePositionResult position_result = dut.FindRoutePosition(road_position);

  EXPECT_TRUE(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.));
}

TEST_F(RouteWithOnePhaseTest,
       FindRoutePositionByRoadPositionWhenRoadPositionIsInRouteBecomesFindPhasePositionByRoadPosition) {
  const api::LanePositionResult kLaneBPositionResult{api::LanePosition{10., 2.5, 0.}, api::InertialPosition{1., 0., 0.},
                                                     0.};
  EXPECT_CALL(lane_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneBPositionResult));
  const api::RoadPosition road_position{&lane_b_, api::LanePosition{10., 2.5, 0.}};
  const Route dut({*phase_a_}, road_network_.get());
  const RoutePositionResult kExpectedRoutePositionResult{
      kPhaseAIndex, PhasePositionResult{1, kLaneBPositionResult.lane_position, kLaneBPositionResult.nearest_position,
                                        kLaneBPositionResult.distance}};

  const RoutePositionResult position_result = dut.FindRoutePosition(road_position);

  EXPECT_TRUE(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.));
}

// Models the following Route:
//
// <pre>
//
//      Phase A        Phase B
//  x------AA------*------BA------*
//  *------AB------x
//
// </pre>
//
// Where:
// - * : is an entry / exit point of the Phase.
// - x : is the extent of the api::LaneSRange.
// - Phase A, api::LaneSRange AA starts at (0., 0., 0.) in the Inertial Frame.
// - Phase A, api::LaneSRange AB starts at (0., -5., 0.) in the Inertial Frame.
// - Phase A, api::LaneSRange AA ends at (100., 0., 0.) in the Inertial Frame.
// - Phase A, api::LaneSRange AB ends at (100., -5., 0.) in the Inertial Frame.
// - Phase B, api::LaneSRange BA starts at (100., 0., 0.) in the Inertial Frame.
// - Phase B, api::LaneSRange BA ends at (200., 0., 0.) in the Inertial Frame.
class RouteWithTwoPhasesTest : public RouteBaseTest {
 public:
  static constexpr int kPhaseAIndex{0};
  static constexpr int kPhaseBIndex{1};
  static constexpr int kLaneSRangeAAIndex{0};
  static constexpr int kLaneSRangeABIndex{1};
  static constexpr int kLaneSRangeBAIndex{0};
  static constexpr double kLaneSRangeTolerance{1e-3};
  static constexpr double kLaneAALength{100.};
  static constexpr double kLaneABLength{100.};
  static constexpr double kLaneBALength{50.};

  const api::SegmentId kSegmentAId{"segment_a"};
  const api::SegmentId kSegmentBId{"segment_b"};
  const api::LaneId kLaneIdAA{"lane_a_a"};
  const api::LaneId kLaneIdAB{"lane_a_b"};
  const api::LaneId kLaneIdBA{"lane_b_a"};
  const api::RBounds kLaneBounds{-2.5, 2.5};
  const api::LaneSRange kLaneSRangeAA{kLaneIdAA, api::SRange{0., kLaneAALength}};
  const api::LaneSRange kLaneSRangeAB{kLaneIdAB, api::SRange{0., kLaneABLength}};
  const api::LaneSRange kLaneSRangeBA{kLaneIdBA, api::SRange{0., kLaneBALength}};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseA{kLaneSRangeAA, kLaneSRangeAB};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseB{kLaneSRangeBA};
  const api::LanePosition kStartLanePositionPhaseA{0., 0., 0.};
  const api::LanePosition kEndLanePositionPhaseA{kLaneAALength, 0., 0.};
  const api::LanePosition kStartLanePositionPhaseB{0., 0., 0.};
  const api::LanePosition kEndLanePositionPhaseB{kLaneBALength, 0., 0.};

  void SetUp() override {
    RouteBaseTest::SetUp();

    EXPECT_CALL(segment_a_, do_id()).WillRepeatedly(Return(kSegmentAId));
    EXPECT_CALL(segment_b_, do_id()).WillRepeatedly(Return(kSegmentBId));
    EXPECT_CALL(lane_a_a_, do_id()).WillRepeatedly(Return(kLaneIdAA));
    EXPECT_CALL(lane_a_a_, do_length()).WillRepeatedly(Return(kLaneAALength));
    EXPECT_CALL(lane_a_a_, do_to_left()).WillRepeatedly(Return(&lane_a_b_));
    EXPECT_CALL(lane_a_a_, do_to_right()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(lane_a_a_, do_segment()).WillRepeatedly(Return(&segment_a_));
    EXPECT_CALL(lane_a_a_, do_lane_bounds(_)).WillRepeatedly(Return(kLaneBounds));
    EXPECT_CALL(lane_a_b_, do_id()).WillRepeatedly(Return(kLaneIdAB));
    EXPECT_CALL(lane_a_b_, do_length()).WillRepeatedly(Return(kLaneABLength));
    EXPECT_CALL(lane_a_b_, do_to_left()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(lane_a_b_, do_to_right()).WillRepeatedly(Return(&lane_a_a_));
    EXPECT_CALL(lane_a_b_, do_segment()).WillRepeatedly(Return(&segment_a_));
    EXPECT_CALL(lane_a_b_, do_lane_bounds(_)).WillRepeatedly(Return(kLaneBounds));
    EXPECT_CALL(lane_b_a_, do_id()).WillRepeatedly(Return(kLaneIdBA));
    EXPECT_CALL(lane_b_a_, do_length()).WillRepeatedly(Return(kLaneBALength));
    EXPECT_CALL(lane_b_a_, do_to_left()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(lane_b_a_, do_to_right()).WillRepeatedly(Return(nullptr));
    EXPECT_CALL(lane_b_a_, do_segment()).WillRepeatedly(Return(&segment_b_));
    EXPECT_CALL(lane_b_a_, do_lane_bounds(_)).WillRepeatedly(Return(kLaneBounds));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdAA)).WillRepeatedly(Return(&lane_a_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdAB)).WillRepeatedly(Return(&lane_a_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdBA)).WillRepeatedly(Return(&lane_b_a_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseA{
        api::RoadPosition{&lane_a_b_, kStartLanePositionPhaseA}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseA{api::RoadPosition{&lane_a_a_, kEndLanePositionPhaseA}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseB{
        api::RoadPosition{&lane_b_a_, kStartLanePositionPhaseB}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseB{api::RoadPosition{&lane_b_a_, kEndLanePositionPhaseB}};
    phase_a_ = std::make_unique<Phase>(kPhaseAIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseA,
                                       kEndRoadPositionsPhaseA, kLaneSRangesPhaseA, road_network_.get());
    phase_b_ = std::make_unique<Phase>(kPhaseBIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseB,
                                       kEndRoadPositionsPhaseB, kLaneSRangesPhaseB, road_network_.get());
  }

  LaneMock lane_a_a_;
  LaneMock lane_a_b_;
  LaneMock lane_b_a_;
  SegmentMock segment_a_;
  SegmentMock segment_b_;
  std::unique_ptr<Phase> phase_a_;
  std::unique_ptr<Phase> phase_b_;
};

TEST_F(RouteWithTwoPhasesTest, SizeIsTwo) {
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());

  EXPECT_EQ(2, dut.size());
}

TEST_F(RouteWithTwoPhasesTest, GetCanIndexPhases) {
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());

  EXPECT_EQ(kPhaseAIndex, dut.Get(kPhaseAIndex).index());
  EXPECT_EQ(kPhaseBIndex, dut.Get(kPhaseBIndex).index());
}

TEST_F(RouteWithTwoPhasesTest, StartRoutePositionIsOnLaneAB) {
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());

  EXPECT_EQ(&lane_a_b_, dut.start_route_position().lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(kStartLanePositionPhaseA, dut.start_route_position().pos, 0.));
}

TEST_F(RouteWithTwoPhasesTest, EndRoutePositionIsOnLaneBA) {
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());

  EXPECT_EQ(&lane_b_a_, dut.end_route_position().lane);
  EXPECT_TRUE(api::test::IsLanePositionClose(kEndLanePositionPhaseB, dut.end_route_position().pos, 0.));
}

TEST_F(RouteWithTwoPhasesTest, FindRoutePositionByInertialPositionIsMappedViaPhaseOntoLaneSRangeAA) {
  // To simplify the modelled situation, please revisit the following diagram where the o
  // indicates where the InertialPosition falls.
  // <pre>
  //
  //      Phase A        Phase B
  //  x-o----AA------*------BA------*
  //  *------AB------x
  //
  // </pre>
  const api::LanePositionResult kLaneAAPositionResult{api::LanePosition{10., 0., 0.},
                                                      api::InertialPosition{10., 0., 0.}, 0.};
  const api::LanePositionResult kLaneABPositionResult{api::LanePosition{10., 2.5, 0.},
                                                      api::InertialPosition{10., -2.5, 0.}, 2.5};
  const api::LanePositionResult kLaneBAPositionResult{api::LanePosition{0., 0., 0.},
                                                      api::InertialPosition{100., 0., 0.}, 90.};
  const api::InertialPosition kInertialPosition{10., 0., 0.};
  EXPECT_CALL(lane_a_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneAAPositionResult));
  EXPECT_CALL(lane_a_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneABPositionResult));
  EXPECT_CALL(lane_b_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneBAPositionResult));
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());
  const RoutePositionResult kExpectedRoutePositionResult{
      kPhaseAIndex, PhasePositionResult{kLaneSRangeAAIndex, kLaneAAPositionResult.lane_position,
                                        kLaneAAPositionResult.nearest_position, kLaneAAPositionResult.distance}};

  const RoutePositionResult position_result = dut.FindRoutePosition(kInertialPosition);

  EXPECT_TRUE(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.));
}

TEST_F(RouteWithTwoPhasesTest, FindRoutePositionByInertialPositionIsMappedViaPhaseOntoLaneSRangeBA) {
  // To simplify the modelled situation, please revisit the following diagram where the o
  // indicates where the InertialPosition falls.
  // <pre>
  //
  //      Phase A        Phase B
  //  x------AA------*-o----BA------*
  //  *------AB------x
  //
  // </pre>
  const api::LanePositionResult kLaneAAPositionResult{api::LanePosition{100., 0., 0.},
                                                      api::InertialPosition{100., 0., 0.}, 10.};
  const api::LanePositionResult kLaneABPositionResult{
      api::LanePosition{100., 2.5, 0.}, api::InertialPosition{100., -2.5, 0.}, std::sqrt(10. * 10. + 2.5 * 2.5)};
  const api::LanePositionResult kLaneBAPositionResult{api::LanePosition{10., 0., 0.},
                                                      api::InertialPosition{110., 0., 0.}, 0.};
  const api::InertialPosition kInertialPosition{110., 0., 0.};
  EXPECT_CALL(lane_a_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneAAPositionResult));
  EXPECT_CALL(lane_a_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneABPositionResult));
  EXPECT_CALL(lane_b_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneBAPositionResult));
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());
  const RoutePositionResult kExpectedRoutePositionResult{
      kPhaseBIndex, PhasePositionResult{kLaneSRangeBAIndex, kLaneBAPositionResult.lane_position,
                                        kLaneBAPositionResult.nearest_position, kLaneBAPositionResult.distance}};

  const RoutePositionResult position_result = dut.FindRoutePosition(kInertialPosition);

  EXPECT_TRUE(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.));
}

TEST_F(RouteWithTwoPhasesTest, FindRoutePositionByRoadPositionWhenRoadPositionIsNotInRouteBecomesByInertialPosition) {
  // To simplify the modelled situation, please revisit the following diagram where the o
  // indicates where the RoadPosition falls, s indicates the position where api::LaneSRange
  // starts, and the m indicates the mapped position. api::LaneSRange AC does not belong to the Route.
  // <pre>
  //
  //      Phase A        Phase B
  //  x------AA------*------BA------*
  //  *------AB------m
  //  s------AC------o
  //
  // </pre>
  const api::LanePositionResult kLaneAAPositionResult{api::LanePosition{100., -2.5, 0.},
                                                      api::InertialPosition{100., -2.5, 0.}, 7.5};
  const api::LanePositionResult kLaneABPositionResult{api::LanePosition{100., -2.5, 0.},
                                                      api::InertialPosition{100., -7.5, 0.}, 2.5};
  const api::LanePositionResult kLaneBAPositionResult{api::LanePosition{0., -2.5, 0.},
                                                      api::InertialPosition{100., -2.5, 0.}, 7.5};
  const api::InertialPosition kInertialPosition{100., -10., 0.};
  EXPECT_CALL(lane_a_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneAAPositionResult));
  EXPECT_CALL(lane_a_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneABPositionResult));
  EXPECT_CALL(lane_b_a_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneBAPositionResult));
  LaneMock lane_a_c;
  EXPECT_CALL(lane_a_c, do_id()).WillRepeatedly(Return(api::LaneId("lane_a_c")));
  EXPECT_CALL(lane_a_c, DoToInertialPosition(_)).WillRepeatedly(Return(kInertialPosition));
  const api::RoadPosition road_position{&lane_a_c, api::LanePosition{100., 0., 0.}};
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());
  const RoutePositionResult kExpectedRoutePositionResult{
      kPhaseAIndex, PhasePositionResult{kLaneSRangeABIndex, kLaneABPositionResult.lane_position,
                                        kLaneABPositionResult.nearest_position, kLaneABPositionResult.distance}};

  const RoutePositionResult position_result = dut.FindRoutePosition(road_position);

  EXPECT_TRUE(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.));
}

TEST_F(RouteWithTwoPhasesTest,
       FindRoutePositionByRoadPositionWhenRoadPositionIsInRouteBecomesFindPhasePositionByRoadPosition) {
  // To simplify the modelled situation, please revisit the following diagram where the o
  // indicates where the RoadPosition falls.
  // <pre>
  //
  //      Phase A        Phase B
  //  x------AA------*------BA------*
  //  *------o---AB--x
  //
  // </pre>
  const api::LanePositionResult kLaneABPositionResult{api::LanePosition{50., 2.5, 0.},
                                                      api::InertialPosition{50., -2.5, 0.}, 0.};
  const api::RoadPosition road_position{&lane_a_b_, api::LanePosition{50., 2.5, 0.}};
  EXPECT_CALL(lane_a_b_, DoToLanePosition(_)).WillRepeatedly(Return(kLaneABPositionResult));
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());
  const RoutePositionResult kExpectedRoutePositionResult{
      kPhaseAIndex, PhasePositionResult{kLaneSRangeABIndex, kLaneABPositionResult.lane_position,
                                        kLaneABPositionResult.nearest_position, kLaneABPositionResult.distance}};

  const RoutePositionResult position_result = dut.FindRoutePosition(road_position);

  EXPECT_TRUE(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.));
}

}  // namespace
}  // namespace test
}  // namespace routing
}  // namespace maliput
