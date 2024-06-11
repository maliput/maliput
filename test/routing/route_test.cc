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

#include "assert_compare.h"
#include "maliput/api/compare.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/common/assertion_error.h"
#include "maliput/routing/compare.h"
#include "maliput/routing/lane_s_range_relation.h"
#include "maliput/routing/phase.h"
#include "maliput/routing/route_position_result.h"
#include "road_network_mocks.h"

namespace maliput {
namespace routing {
namespace test {
namespace {

using maliput::test::AssertCompare;
using maliput::test::IdIndexMock;
using maliput::test::LaneMock;
using maliput::test::MakeMockedRoadNetwork;
using maliput::test::RoadGeometryMock;
using maliput::test::SegmentMock;
using ::testing::_;
using ::testing::Matcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;
using ::testing::Return;
using ::testing::ReturnRef;

// Initializes the @p lane for the road.
void SetUpLane(LaneMock* lane, const api::LaneId& lane_id, const LaneMock* to_left_lane = nullptr,
               const LaneMock* to_right_lane = nullptr, const SegmentMock* segment = nullptr,
               const api::RBounds& lane_bounds = api::RBounds(-2.5, 2.5), double length = 100.) {
  EXPECT_CALL(*lane, do_id()).WillRepeatedly(Return(lane_id));
  EXPECT_CALL(*lane, do_to_left()).WillRepeatedly(Return(to_left_lane));
  EXPECT_CALL(*lane, do_to_right()).WillRepeatedly(Return(to_right_lane));
  EXPECT_CALL(*lane, do_segment()).WillRepeatedly(Return(segment));
  EXPECT_CALL(*lane, do_lane_bounds(_)).WillRepeatedly(Return(lane_bounds));
  EXPECT_CALL(*lane, do_length()).WillRepeatedly(Return(length));
}

class RouteBaseTest : public ::testing::Test {
 public:
  static constexpr SegmentMock* kNullSegmentPtr{nullptr};
  static constexpr const LaneMock* kNullLeftLanePtr{nullptr};
  static constexpr const LaneMock* kNullRightLanePtr{nullptr};
  const api::RBounds kLaneBounds{-2.5, 2.5};

  void SetUp() override {
    road_network_ = MakeMockedRoadNetwork();
    road_geometry_ptr_ =
        const_cast<RoadGeometryMock*>(static_cast<const RoadGeometryMock*>(road_network_->road_geometry()));
  }

 protected:
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
  SetUpLane(&lane_a, kLaneIdA, &lane_b, kNullRightLanePtr, kNullSegmentPtr, kLaneBounds, kLaneALength);
  SetUpLane(&lane_b, kLaneIdB, kNullLeftLanePtr, &lane_a, kNullSegmentPtr, kLaneBounds, kLaneBLength);
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
  SetUpLane(&lane_a, kLaneIdA, &lane_b, kNullRightLanePtr, kNullSegmentPtr, kLaneBounds, kLaneALength);
  SetUpLane(&lane_b, kLaneIdB, kNullLeftLanePtr, &lane_a, kNullSegmentPtr, kLaneBounds, kLaneBLength);
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
    SetUpLane(&lane_a_, kLaneIdA, &lane_b_, kNullRightLanePtr, kNullSegmentPtr, kLaneBounds, kLaneALength);
    SetUpLane(&lane_b_, kLaneIdB, kNullLeftLanePtr, &lane_a_, kNullSegmentPtr, kLaneBounds, kLaneBLength);
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositions{api::RoadPosition{&lane_a_, kStartRouteLanePosition}};
    const std::vector<api::RoadPosition> kEndRoadPositions{api::RoadPosition{&lane_b_, kEndRouteLanePosition}};
    const std::vector<api::LaneSRange> kLaneSRanges{kLaneSRangeA, kLaneSRangeB};

    phase_a_ = std::make_unique<Phase>(kPhaseAIndex, kLaneSRangeTolerance, kStartRoadPositions, kEndRoadPositions,
                                       kLaneSRanges, road_network_.get());
  }

 protected:
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
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(kStartRouteLanePosition, phase.start_positions()[0].pos, 0.)));
  EXPECT_EQ(phase.end_positions().size(), phase_a_->end_positions().size());
  EXPECT_EQ(phase.end_positions()[0].lane, &lane_b_);
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(kEndRouteLanePosition, phase.end_positions()[0].pos, 0.)));
  EXPECT_EQ(phase.lane_s_ranges().size(), phase_a_->lane_s_ranges().size());
  EXPECT_TRUE(AssertCompare(IsEqual(phase_a_->lane_s_ranges()[0], phase.lane_s_ranges()[0])));
  EXPECT_TRUE(AssertCompare(IsEqual(phase_a_->lane_s_ranges()[1], phase.lane_s_ranges()[1])));
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
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(kStartRouteLanePosition, start_road_position.pos, 0.)));
}

TEST_F(RouteAccessorsTest, EndRoutePositionIsOnLaneSRangeB) {
  const Route dut({*phase_a_}, road_network_.get());

  const api::RoadPosition& end_road_position = dut.end_route_position();

  EXPECT_EQ(end_road_position.lane, &lane_b_);
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(kEndRouteLanePosition, end_road_position.pos, 0.)));
}

class RouteWithOnePhaseTest : public RouteAccessorsTest {
 public:
  const api::SegmentId kSegmentAId{"segment_a"};

  void SetUp() override {
    RouteAccessorsTest::SetUp();
    EXPECT_CALL(segment_a_, do_id()).WillRepeatedly(Return(kSegmentAId));
    EXPECT_CALL(lane_a_, do_segment()).WillRepeatedly(Return(&segment_a_));
    EXPECT_CALL(lane_b_, do_segment()).WillRepeatedly(Return(&segment_a_));
  }

 protected:
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

  EXPECT_TRUE(AssertCompare(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.)));
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

  EXPECT_TRUE(AssertCompare(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.)));
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

  EXPECT_TRUE(AssertCompare(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.)));
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
    SetUpLane(&lane_a_a_, kLaneIdAA, &lane_a_b_, kNullRightLanePtr, &segment_a_, kLaneBounds, kLaneAALength);
    SetUpLane(&lane_a_b_, kLaneIdAB, kNullLeftLanePtr, &lane_a_a_, &segment_a_, kLaneBounds, kLaneABLength);
    SetUpLane(&lane_b_a_, kLaneIdBA, kNullLeftLanePtr, kNullRightLanePtr, &segment_b_, kLaneBounds, kLaneBALength);
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

 protected:
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
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(kStartLanePositionPhaseA, dut.start_route_position().pos, 0.)));
}

TEST_F(RouteWithTwoPhasesTest, EndRoutePositionIsOnLaneBA) {
  const Route dut({*phase_a_, *phase_b_}, road_network_.get());

  EXPECT_EQ(&lane_b_a_, dut.end_route_position().lane);
  EXPECT_TRUE(AssertCompare(IsLanePositionClose(kEndLanePositionPhaseB, dut.end_route_position().pos, 0.)));
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

  EXPECT_TRUE(AssertCompare(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.)));
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

  EXPECT_TRUE(AssertCompare(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.)));
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

  EXPECT_TRUE(AssertCompare(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.)));
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

  EXPECT_TRUE(AssertCompare(IsRoutePositionResultClose(kExpectedRoutePositionResult, position_result, 0.)));
}

// Models the following Route:
//
// <pre>
//
//  x------A>-------x
//  x------B>-------x------E>-------x------H>-------x
//  x------C>-------x------F>-------x------I>-------x
//  x------D>-------x------G>-------x
//
// </pre>
//
// The Inertial Frame origin is at the start of Lane D. Each Lane has a width of 5m,
// 2.5m to each side of the centerline. Thus, Lane D, G are aligned with y=0;
// Lane C, F, and I are aligned with y=5; Lane B, E and H are aligned with y=10;
// Lane A is aligned with y=15.
//
// For more details, refer to LaneSRangeRelation documentation.
class RouteLaneSRelationTest : public RouteBaseTest {
 public:
  static constexpr int kPhaseZeroIndex{0};
  static constexpr int kPhaseOneIndex{1};
  static constexpr int kPhaseTwoIndex{2};
  static constexpr double kLinearTolerance{1e-3};
  static constexpr double kLaneSRangeTolerance{1e-3};
  static constexpr double kLaneLength{100.};

  const api::SegmentId kSegmentZeroId{"segment_0"};
  const api::SegmentId kSegmentOneId{"segment_1"};
  const api::SegmentId kSegmentTwoId{"segment_2"};

  const api::LaneId kLaneIdA{"lane_a"};
  const api::LaneId kLaneIdB{"lane_b"};
  const api::LaneId kLaneIdC{"lane_c"};
  const api::LaneId kLaneIdD{"lane_d"};
  const api::LaneId kLaneIdE{"lane_e"};
  const api::LaneId kLaneIdF{"lane_f"};
  const api::LaneId kLaneIdG{"lane_g"};
  const api::LaneId kLaneIdH{"lane_h"};
  const api::LaneId kLaneIdI{"lane_i"};
  const api::LaneId kLaneIdUnknown{"lane_unknown"};
  const api::LaneSRange kLaneSRangeA{kLaneIdA, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeB{kLaneIdB, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeC{kLaneIdC, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeD{kLaneIdD, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeE{kLaneIdE, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeF{kLaneIdF, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeG{kLaneIdG, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeH{kLaneIdH, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeI{kLaneIdI, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeUnknown{kLaneIdUnknown, api::SRange{0., kLaneLength}};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeD, kLaneSRangeC, kLaneSRangeB, kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeG, kLaneSRangeF, kLaneSRangeE};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseTwo{kLaneSRangeI, kLaneSRangeH};
  const api::LanePosition kStartLanePositionPhase{0., 0., 0.};
  const api::LanePosition kEndLanePositionPhase{kLaneLength, 0., 0.};
  const api::Rotation kOrientation{};

  void SetUp() override {
    RouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    EXPECT_CALL(segment_two_, do_id()).WillRepeatedly(Return(kSegmentTwoId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, &lane_b_, &segment_zero_, kLaneBounds, kLaneLength);
    SetUpLane(&lane_b_, kLaneIdB, &lane_a_, &lane_c_, &segment_zero_, kLaneBounds, kLaneLength);
    SetUpLane(&lane_c_, kLaneIdC, &lane_b_, &lane_d_, &segment_zero_, kLaneBounds, kLaneLength);
    SetUpLane(&lane_d_, kLaneIdD, &lane_c_, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    // Segment one.
    SetUpLane(&lane_e_, kLaneIdE, nullptr, &lane_f_, &segment_one_, kLaneBounds, kLaneLength);
    SetUpLane(&lane_f_, kLaneIdF, &lane_e_, &lane_g_, &segment_one_, kLaneBounds, kLaneLength);
    SetUpLane(&lane_g_, kLaneIdG, &lane_f_, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    // Segment two.
    SetUpLane(&lane_h_, kLaneIdH, nullptr, &lane_i_, &segment_two_, kLaneBounds, kLaneLength);
    SetUpLane(&lane_i_, kLaneIdI, &lane_h_, nullptr, &segment_two_, kLaneBounds, kLaneLength);
    // Unknown lane.
    SetUpLane(&lane_unknown_, kLaneIdUnknown, nullptr, nullptr, nullptr, kLaneBounds, kLaneLength);
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdD)).WillRepeatedly(Return(&lane_d_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdE)).WillRepeatedly(Return(&lane_e_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdF)).WillRepeatedly(Return(&lane_f_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdG)).WillRepeatedly(Return(&lane_g_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdH)).WillRepeatedly(Return(&lane_h_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdI)).WillRepeatedly(Return(&lane_i_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdUnknown)).WillRepeatedly(Return(&lane_unknown_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_b_, kEndLanePositionPhase},
                                                                    api::RoadPosition{&lane_c_, kEndLanePositionPhase},
                                                                    api::RoadPosition{&lane_d_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_e_, kStartLanePositionPhase}, api::RoadPosition{&lane_f_, kStartLanePositionPhase},
        api::RoadPosition{&lane_g_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_e_, kEndLanePositionPhase},
                                                                   api::RoadPosition{&lane_f_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseTwo{
        api::RoadPosition{&lane_h_, kStartLanePositionPhase}, api::RoadPosition{&lane_i_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseTwo{api::RoadPosition{&lane_i_, kEndLanePositionPhase}};
    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    phase_two_ = std::make_unique<Phase>(kPhaseTwoIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseTwo,
                                         kEndRoadPositionsPhaseTwo, kLaneSRangesPhaseTwo, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }

 protected:
  LaneMock lane_a_;
  LaneMock lane_b_;
  LaneMock lane_c_;
  LaneMock lane_d_;
  LaneMock lane_e_;
  LaneMock lane_f_;
  LaneMock lane_g_;
  LaneMock lane_h_;
  LaneMock lane_i_;
  LaneMock lane_unknown_;
  SegmentMock segment_zero_;
  SegmentMock segment_one_;
  SegmentMock segment_two_;
  std::unique_ptr<Phase> phase_zero_;
  std::unique_ptr<Phase> phase_one_;
  std::unique_ptr<Phase> phase_two_;
};

TEST_F(RouteLaneSRelationTest, EvaluateComputeLaneSRangeRelationSameLaneSRanges) {
  const Route dut({*phase_zero_, *phase_one_, *phase_two_}, road_network_.get());

  EXPECT_EQ(LaneSRangeRelation::kUnknown, dut.ComputeLaneSRangeRelation(kLaneSRangeUnknown, kLaneSRangeA));

  EXPECT_EQ(LaneSRangeRelation::kUnrelated, dut.ComputeLaneSRangeRelation(kLaneSRangeH, kLaneSRangeA));

  EXPECT_EQ(LaneSRangeRelation::kCoincident, dut.ComputeLaneSRangeRelation(kLaneSRangeA, kLaneSRangeA));

  EXPECT_EQ(LaneSRangeRelation::kAdjacentLeft, dut.ComputeLaneSRangeRelation(kLaneSRangeB, kLaneSRangeA));

  EXPECT_EQ(LaneSRangeRelation::kLeft, dut.ComputeLaneSRangeRelation(kLaneSRangeD, kLaneSRangeA));

  EXPECT_EQ(LaneSRangeRelation::kAdjacentRight, dut.ComputeLaneSRangeRelation(kLaneSRangeA, kLaneSRangeB));

  EXPECT_EQ(LaneSRangeRelation::kRight, dut.ComputeLaneSRangeRelation(kLaneSRangeA, kLaneSRangeD));

  EXPECT_CALL(lane_c_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{kLaneLength, 5., 0.}));
  EXPECT_CALL(lane_f_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{kLaneLength, 5., 0.}));
  EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));
  EXPECT_CALL(lane_f_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));
  EXPECT_EQ(LaneSRangeRelation::kSucceedingStraight, dut.ComputeLaneSRangeRelation(kLaneSRangeC, kLaneSRangeF));

  EXPECT_CALL(lane_d_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
  EXPECT_CALL(lane_e_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{kLaneLength, 10., 0.}));
  EXPECT_CALL(lane_d_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));
  EXPECT_CALL(lane_e_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));

  EXPECT_EQ(LaneSRangeRelation::kSucceedingLeft, dut.ComputeLaneSRangeRelation(kLaneSRangeD, kLaneSRangeE));

  EXPECT_EQ(LaneSRangeRelation::kSucceedingLeft, dut.ComputeLaneSRangeRelation(kLaneSRangeD, kLaneSRangeF));

  EXPECT_CALL(lane_b_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{kLaneLength, 15., 0.}));
  EXPECT_CALL(lane_g_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
  EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));
  EXPECT_CALL(lane_g_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));

  EXPECT_EQ(LaneSRangeRelation::kSucceedingRight, dut.ComputeLaneSRangeRelation(kLaneSRangeB, kLaneSRangeF));

  EXPECT_EQ(LaneSRangeRelation::kSucceedingRight, dut.ComputeLaneSRangeRelation(kLaneSRangeB, kLaneSRangeG));

  EXPECT_CALL(lane_e_, DoToInertialPosition(_))
      .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 10., 0.}));
  EXPECT_CALL(lane_h_, DoToInertialPosition(_))
      .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 10., 0.}));
  EXPECT_CALL(lane_e_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));
  EXPECT_CALL(lane_h_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));

  EXPECT_EQ(LaneSRangeRelation::kPrecedingStraight, dut.ComputeLaneSRangeRelation(kLaneSRangeH, kLaneSRangeE));

  EXPECT_CALL(lane_i_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 5., 0.}));
  EXPECT_CALL(lane_i_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));

  EXPECT_EQ(LaneSRangeRelation::kPrecedingLeft, dut.ComputeLaneSRangeRelation(kLaneSRangeI, kLaneSRangeE));

  EXPECT_CALL(lane_f_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 5., 0.}));
  EXPECT_CALL(lane_g_, DoToInertialPosition(_)).WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 0., 0.}));
  EXPECT_CALL(lane_f_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));
  EXPECT_CALL(lane_g_, DoGetOrientation(_)).WillRepeatedly(Return(kOrientation));

  EXPECT_EQ(LaneSRangeRelation::kPrecedingRight, dut.ComputeLaneSRangeRelation(kLaneSRangeH, kLaneSRangeF));

  EXPECT_EQ(LaneSRangeRelation::kPrecedingRight, dut.ComputeLaneSRangeRelation(kLaneSRangeH, kLaneSRangeG));
}

// The following set of tests evaluate specific conditions of the topology of the graphs.
// That enables validating the internal routing to verify the resulting api::LaneSRoute is correct.

/// Wraps api::LanePosition comparison into a MatcherInterface.
class LanePositionMatcher : public MatcherInterface<const api::LanePosition&> {
 public:
  explicit LanePositionMatcher(const api::LanePosition& lane_position) : lane_position_(lane_position) {}

  bool MatchAndExplain(const api::LanePosition& other_lane_position, MatchResultListener*) const override {
    return lane_position_.srh() == other_lane_position.srh();
  }

  void DescribeTo(std::ostream* os) const override {
    *os << "is equal to api::LanePosition{" << lane_position_.s() << ", " << lane_position_.r() << ", "
        << lane_position_.h() << "}.";
  }

 private:
  const api::LanePosition& lane_position_;
};

// Creates a Matcher to validate api::LanePositions.
Matcher<const api::LanePosition&> Matches(const api::LanePosition& lane_position) {
  return MakeMatcher(new LanePositionMatcher(lane_position));
}

// Base test fixture class to validate api::LaneSRoute.
class RouteComputeLaneSRouteBaseTest : public RouteBaseTest {
 public:
  static constexpr int kPhaseZeroIndex{0};
  static constexpr int kPhaseOneIndex{1};
  static constexpr double kLinearTolerance{1e-3};
  static constexpr double kLaneSRangeTolerance{1e-3};
  static constexpr double kLaneLength{100.};

  const api::SegmentId kSegmentZeroId{"segment_0"};
  const api::SegmentId kSegmentOneId{"segment_1"};

  const api::LaneId kLaneIdA{"lane_a"};
  const api::LaneId kLaneIdB{"lane_b"};
  const api::LaneId kLaneIdC{"lane_c"};
  const api::LaneId kLaneIdD{"lane_d"};

  const api::LaneSRange kLaneSRangeA{kLaneIdA, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeB{kLaneIdB, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeC{kLaneIdC, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeD{kLaneIdD, api::SRange{0., kLaneLength}};

  const api::LanePosition kStartLanePositionPhase{0., 0., 0.};
  const api::LanePosition kEndLanePositionPhase{kLaneLength, 0., 0.};
  const api::LanePosition kQueryLanePosition{kLaneLength / 2., 0., 0.};
  const api::Rotation kRotation{};

 protected:
  LaneMock lane_a_;
  LaneMock lane_b_;
  LaneMock lane_c_;
  LaneMock lane_d_;
  SegmentMock segment_zero_;
  SegmentMock segment_one_;
  std::unique_ptr<Phase> phase_zero_;
  std::unique_ptr<Phase> phase_one_;
  std::unique_ptr<Phase> phase_two_;
};

// Case A:
// - Feature under evaluation: can map a LaneSRoute straight backwards.
// - Topology:
// <pre>
//
// x----A>----x----B>----x----C>----x
//
// </pre>
class RouteComputeLaneSRouteCaseATest : public RouteComputeLaneSRouteBaseTest {
 public:
  static constexpr int kPhaseTwoIndex{2};

  const api::SegmentId kSegmentTwoId{"segment_2"};

  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeB};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseTwo{kLaneSRangeC};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    EXPECT_CALL(segment_two_, do_id()).WillRepeatedly(Return(kSegmentTwoId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_b_, kLaneIdB, nullptr, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment two.
    SetUpLane(&lane_c_, kLaneIdC, nullptr, nullptr, &segment_two_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_a_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_b_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_b_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseTwo{
        api::RoadPosition{&lane_c_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseTwo{api::RoadPosition{&lane_c_, kEndLanePositionPhase}};

    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    phase_two_ = std::make_unique<Phase>(kPhaseTwoIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseTwo,
                                         kEndRoadPositionsPhaseTwo, kLaneSRangesPhaseTwo, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }

 protected:
  SegmentMock segment_two_;
  std::unique_ptr<Phase> phase_two_;
};

TEST_F(RouteComputeLaneSRouteCaseATest, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC});
  const Route dut({*phase_zero_, *phase_one_, *phase_two_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
}

// Case B.1:
// - Feature under evaluation: can move to the right at the beginning in the same Phase.
// - Topology:
// <pre>
//
// x----A>----*
// *----B>----x----C>----x
//
// </pre>
class RouteComputeLaneSRouteCaseB1Test : public RouteComputeLaneSRouteBaseTest {
 public:
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeB, kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeC};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, &lane_b_, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_b_, kLaneIdB, &lane_a_, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -5., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_c_, kLaneIdC, nullptr, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -5., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_b_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_c_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_c_, kEndLanePositionPhase}};

    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }
};

TEST_F(RouteComputeLaneSRouteCaseB1Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC});
  const Route dut({*phase_zero_, *phase_one_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
}

// Case B.2:
// - Feature under evaluation: can perform multiple lane switches to the right at the beginning in the same Phase.
// - Topology:
// <pre>
//
// x----A>----*
// x----B>----*
// *----C>----x----D>----x
//
// </pre>
class RouteComputeLaneSRouteCaseB2Test : public RouteComputeLaneSRouteBaseTest {
 public:
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeC, kLaneSRangeB, kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeD};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, &lane_b_, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_b_, kLaneIdB, &lane_a_, &lane_c_, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -5., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_c_, kLaneIdC, &lane_b_, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -10., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_d_, kLaneIdD, nullptr, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_d_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -10., 0.}));
    EXPECT_CALL(lane_d_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdD)).WillRepeatedly(Return(&lane_d_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_c_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_d_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_d_, kEndLanePositionPhase}};

    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }
};

TEST_F(RouteComputeLaneSRouteCaseB2Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionD{&lane_d_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromD({kLaneSRangeD});
  const Route dut({*phase_zero_, *phase_one_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromD, dut.ComputeLaneSRoute(kRoadPositionD))));
}

// Case C.1:
// - Feature under evaluation: can move to the left at the beginning in the same Phase.
// - Topology:
// <pre>
//
// *----B>----x----C>----x
// x----A>----*
//
// </pre>
class RouteComputeLaneSRouteCaseC1Test : public RouteComputeLaneSRouteBaseTest {
 public:
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA, kLaneSRangeB};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeC};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, &lane_b_, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -5., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_b_, kLaneIdB, nullptr, &lane_a_, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_c_, kLaneIdC, nullptr, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_b_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_c_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_c_, kEndLanePositionPhase}};

    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }
};

TEST_F(RouteComputeLaneSRouteCaseC1Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC});
  const Route dut({*phase_zero_, *phase_one_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
}

// Case C.2:
// - Feature under evaluation: can move multiple lane switches to the left at the beginning in the same Phase.
// - Topology:
// <pre>
//
// *----C>----x----D>----x
// x----B>----*
// x----A>----*
//
// </pre>
class RouteComputeLaneSRouteCaseC2Test : public RouteComputeLaneSRouteBaseTest {
 public:
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA, kLaneSRangeB, kLaneSRangeC};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeD};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, &lane_b_, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -10., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_b_, kLaneIdB, &lane_c_, &lane_a_, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -5., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_c_, kLaneIdC, nullptr, &lane_b_, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_d_, kLaneIdD, nullptr, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_d_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_d_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdD)).WillRepeatedly(Return(&lane_d_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_c_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_d_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_d_, kEndLanePositionPhase}};

    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }
};

TEST_F(RouteComputeLaneSRouteCaseC2Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionD{&lane_d_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromD({kLaneSRangeD});
  const Route dut({*phase_zero_, *phase_one_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromD, dut.ComputeLaneSRoute(kRoadPositionD))));
}

// Case D.1:
// - Feature under evaluation: can move to the right at the end in the same Phase.
// - Topology:
// <pre>
//
// x----A>----x----B>----*
//            *----C>----x
//
// </pre>
class RouteComputeLaneSRouteCaseD1Test : public RouteComputeLaneSRouteBaseTest {
 public:
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeC, kLaneSRangeB};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_b_, kLaneIdB, &lane_c_, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_c_, kLaneIdC, nullptr, &lane_b_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -5., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_a_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_b_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_c_, kEndLanePositionPhase}};
    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }
};

TEST_F(RouteComputeLaneSRouteCaseD1Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC});
  const Route dut({*phase_zero_, *phase_one_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
}

// Case D.2:
// - Feature under evaluation: can move multiple times to the right at the end in the same Phase.
// - Topology:
// <pre>
//
// x----A>----x----B>----*
//            *----C>----*
//            *----D>----x
//
// </pre>
class RouteComputeLaneSRouteCaseD2Test : public RouteComputeLaneSRouteBaseTest {
 public:
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeD, kLaneSRangeC, kLaneSRangeB};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_b_, kLaneIdB, &lane_c_, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_c_, kLaneIdC, &lane_d_, &lane_b_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -5., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_d_, kLaneIdD, nullptr, &lane_c_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_d_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, -10., 0.}));
    EXPECT_CALL(lane_d_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdD)).WillRepeatedly(Return(&lane_d_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_a_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_b_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_d_, kEndLanePositionPhase}};
    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }
};

TEST_F(RouteComputeLaneSRouteCaseD2Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionD{&lane_d_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromD({kLaneSRangeD});
  const Route dut({*phase_zero_, *phase_one_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromD, dut.ComputeLaneSRoute(kRoadPositionD))));
}

// Case E.1:
// - Feature under evaluation: can move to the left at the end in the same Phase.
// - Topology:
// <pre>
//
//            *----C>----x
// *----A>----x----B>----*
//
// </pre>
class RouteComputeLaneSRouteCaseE1Test : public RouteComputeLaneSRouteBaseTest {
 public:
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeB, kLaneSRangeC};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_b_, kLaneIdB, nullptr, &lane_c_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_c_, kLaneIdC, &lane_b_, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_a_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_b_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_c_, kEndLanePositionPhase}};
    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }
};

TEST_F(RouteComputeLaneSRouteCaseE1Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC});
  const Route dut({*phase_zero_, *phase_one_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
}

// Case E.2:
// - Feature under evaluation: can move multiple times to the left at the end in the same Phase.
// - Topology:
// <pre>
//
//            *----D>----x
//            *----C>----*
// *----A>----x----B>----*
//
// </pre>
class RouteComputeLaneSRouteCaseE2Test : public RouteComputeLaneSRouteBaseTest {
 public:
  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeB, kLaneSRangeC, kLaneSRangeD};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_b_, kLaneIdB, nullptr, &lane_c_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_c_, kLaneIdC, &lane_b_, &lane_d_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_d_, kLaneIdD, &lane_c_, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_d_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 10., 0.}));
    EXPECT_CALL(lane_d_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdD)).WillRepeatedly(Return(&lane_d_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_a_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_b_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_d_, kEndLanePositionPhase}};
    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }
};

TEST_F(RouteComputeLaneSRouteCaseE2Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionD{&lane_d_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromB({kLaneSRangeB, kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromC({kLaneSRangeC, kLaneSRangeD});
  const api::LaneSRoute kExpectedLaneSRouteFromD({kLaneSRangeD});
  const Route dut({*phase_zero_, *phase_one_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromD, dut.ComputeLaneSRoute(kRoadPositionD))));
}

// Case F.1:
// - Feature under evaluation: combines all movements.
// - Topology:
// <pre>
//
//            *----C>----x----D>----x----E>----*
// *----A>----x----B>----*          *----F>----x----G>----*
//
// </pre>
class RouteComputeLaneSRouteCaseF1Test : public RouteComputeLaneSRouteBaseTest {
 public:
  static constexpr int kPhaseTwoIndex{2};
  static constexpr int kPhaseThreeIndex{3};
  static constexpr int kPhaseFourIndex{4};
  static constexpr double kLinearTolerance{1e-3};
  static constexpr double kLaneSRangeTolerance{1e-3};
  static constexpr double kLaneLength{100.};

  const api::SegmentId kSegmentTwoId{"segment_2"};
  const api::SegmentId kSegmentThreeId{"segment_3"};
  const api::SegmentId kSegmentFourId{"segment_4"};

  const api::LaneId kLaneIdE{"lane_e"};
  const api::LaneId kLaneIdF{"lane_f"};
  const api::LaneId kLaneIdG{"lane_g"};

  const api::LaneSRange kLaneSRangeE{kLaneIdE, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeF{kLaneIdF, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeG{kLaneIdG, api::SRange{0., kLaneLength}};

  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeB, kLaneSRangeC};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseTwo{kLaneSRangeD};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseThree{kLaneSRangeF, kLaneSRangeE};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseFour{kLaneSRangeG};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    EXPECT_CALL(segment_two_, do_id()).WillRepeatedly(Return(kSegmentTwoId));
    EXPECT_CALL(segment_three_, do_id()).WillRepeatedly(Return(kSegmentThreeId));
    EXPECT_CALL(segment_four_, do_id()).WillRepeatedly(Return(kSegmentFourId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_b_, kLaneIdB, nullptr, &lane_c_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_c_, kLaneIdC, &lane_b_, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment two.
    SetUpLane(&lane_d_, kLaneIdD, nullptr, nullptr, &segment_two_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_d_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_d_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{3. * kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_d_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment three.
    SetUpLane(&lane_e_, kLaneIdE, &lane_f_, nullptr, &segment_three_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_e_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{3. * kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_e_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{4. * kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_e_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_f_, kLaneIdF, nullptr, &lane_e_, &segment_three_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_f_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{3. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_f_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{4. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_f_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment four.
    SetUpLane(&lane_g_, kLaneIdG, nullptr, nullptr, &segment_four_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_g_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{4. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_g_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdD)).WillRepeatedly(Return(&lane_d_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdE)).WillRepeatedly(Return(&lane_e_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdF)).WillRepeatedly(Return(&lane_f_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdG)).WillRepeatedly(Return(&lane_g_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_a_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_b_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_c_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseTwo{
        api::RoadPosition{&lane_d_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseTwo{api::RoadPosition{&lane_d_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseThree{
        api::RoadPosition{&lane_e_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseThree{
        api::RoadPosition{&lane_f_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseFour{
        api::RoadPosition{&lane_g_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseFour{api::RoadPosition{&lane_g_, kEndLanePositionPhase}};
    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    phase_two_ = std::make_unique<Phase>(kPhaseTwoIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseTwo,
                                         kEndRoadPositionsPhaseTwo, kLaneSRangesPhaseTwo, road_network_.get());
    phase_three_ = std::make_unique<Phase>(kPhaseThreeIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseThree,
                                           kEndRoadPositionsPhaseThree, kLaneSRangesPhaseThree, road_network_.get());
    phase_four_ = std::make_unique<Phase>(kPhaseFourIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseFour,
                                          kEndRoadPositionsPhaseFour, kLaneSRangesPhaseFour, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }

 protected:
  LaneMock lane_e_;
  LaneMock lane_f_;
  LaneMock lane_g_;
  SegmentMock segment_two_;
  SegmentMock segment_three_;
  SegmentMock segment_four_;
  std::unique_ptr<Phase> phase_two_;
  std::unique_ptr<Phase> phase_three_;
  std::unique_ptr<Phase> phase_four_;
};

TEST_F(RouteComputeLaneSRouteCaseF1Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionD{&lane_d_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionE{&lane_e_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionF{&lane_f_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionG{&lane_g_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA(
      {kLaneSRangeA, kLaneSRangeB, kLaneSRangeC, kLaneSRangeD, kLaneSRangeE, kLaneSRangeF, kLaneSRangeG});
  const api::LaneSRoute kExpectedLaneSRouteFromB(
      {kLaneSRangeB, kLaneSRangeC, kLaneSRangeD, kLaneSRangeE, kLaneSRangeF, kLaneSRangeG});
  const api::LaneSRoute kExpectedLaneSRouteFromC(
      {kLaneSRangeC, kLaneSRangeD, kLaneSRangeE, kLaneSRangeF, kLaneSRangeG});
  const api::LaneSRoute kExpectedLaneSRouteFromD({kLaneSRangeD, kLaneSRangeE, kLaneSRangeF, kLaneSRangeG});
  const api::LaneSRoute kExpectedLaneSRouteFromE({kLaneSRangeE, kLaneSRangeF, kLaneSRangeG});
  const api::LaneSRoute kExpectedLaneSRouteFromF({kLaneSRangeF, kLaneSRangeG});
  const api::LaneSRoute kExpectedLaneSRouteFromG({kLaneSRangeG});
  const Route dut({*phase_zero_, *phase_one_, *phase_two_, *phase_three_, *phase_four_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromD, dut.ComputeLaneSRoute(kRoadPositionD))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromE, dut.ComputeLaneSRoute(kRoadPositionE))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromF, dut.ComputeLaneSRoute(kRoadPositionF))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromG, dut.ComputeLaneSRoute(kRoadPositionG))));
}

// Case F.2:
// - Feature under evaluation: combines all movements.
// - Topology:
// <pre>
//
//            *----D>----x----E>----x----F>----*
//            *----C>----*          x----G>----*
// *----A>----x----B>----*          *----H>----x----I>----*
//
// </pre>
class RouteComputeLaneSRouteCaseF2Test : public RouteComputeLaneSRouteBaseTest {
 public:
  static constexpr int kPhaseTwoIndex{2};
  static constexpr int kPhaseThreeIndex{3};
  static constexpr int kPhaseFourIndex{4};
  static constexpr double kLinearTolerance{1e-3};
  static constexpr double kLaneSRangeTolerance{1e-3};
  static constexpr double kLaneLength{100.};

  const api::SegmentId kSegmentTwoId{"segment_2"};
  const api::SegmentId kSegmentThreeId{"segment_3"};
  const api::SegmentId kSegmentFourId{"segment_4"};

  const api::LaneId kLaneIdE{"lane_e"};
  const api::LaneId kLaneIdF{"lane_f"};
  const api::LaneId kLaneIdG{"lane_g"};
  const api::LaneId kLaneIdH{"lane_h"};
  const api::LaneId kLaneIdI{"lane_i"};

  const api::LaneSRange kLaneSRangeE{kLaneIdE, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeF{kLaneIdF, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeG{kLaneIdG, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeH{kLaneIdH, api::SRange{0., kLaneLength}};
  const api::LaneSRange kLaneSRangeI{kLaneIdI, api::SRange{0., kLaneLength}};

  const std::vector<api::LaneSRange> kLaneSRangesPhaseZero{kLaneSRangeA};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseOne{kLaneSRangeB, kLaneSRangeC, kLaneSRangeD};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseTwo{kLaneSRangeE};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseThree{kLaneSRangeH, kLaneSRangeG, kLaneSRangeF};
  const std::vector<api::LaneSRange> kLaneSRangesPhaseFour{kLaneSRangeI};

  void SetUp() override {
    RouteComputeLaneSRouteBaseTest::SetUp();
    EXPECT_CALL(segment_zero_, do_id()).WillRepeatedly(Return(kSegmentZeroId));
    EXPECT_CALL(segment_one_, do_id()).WillRepeatedly(Return(kSegmentOneId));
    EXPECT_CALL(segment_two_, do_id()).WillRepeatedly(Return(kSegmentTwoId));
    EXPECT_CALL(segment_three_, do_id()).WillRepeatedly(Return(kSegmentThreeId));
    EXPECT_CALL(segment_four_, do_id()).WillRepeatedly(Return(kSegmentFourId));
    // Segment zero.
    SetUpLane(&lane_a_, kLaneIdA, nullptr, nullptr, &segment_zero_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_a_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_a_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment one.
    SetUpLane(&lane_b_, kLaneIdB, nullptr, &lane_c_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_b_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_c_, kLaneIdC, &lane_b_, &lane_d_, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_c_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_c_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_d_, kLaneIdD, &lane_c_, nullptr, &segment_one_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_d_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{kLaneLength, 10., 0.}));
    EXPECT_CALL(lane_d_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 10., 0.}));
    EXPECT_CALL(lane_d_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment two.
    SetUpLane(&lane_e_, kLaneIdE, nullptr, nullptr, &segment_two_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_e_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{2. * kLaneLength, 10., 0.}));
    EXPECT_CALL(lane_e_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{3. * kLaneLength, 10., 0.}));
    EXPECT_CALL(lane_e_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment three.
    SetUpLane(&lane_f_, kLaneIdF, &lane_g_, nullptr, &segment_three_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_f_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{3. * kLaneLength, 10., 0.}));
    EXPECT_CALL(lane_f_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{4. * kLaneLength, 10., 0.}));
    EXPECT_CALL(lane_f_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_g_, kLaneIdG, &lane_h_, &lane_f_, &segment_three_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_g_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{3. * kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_g_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{4. * kLaneLength, 5., 0.}));
    EXPECT_CALL(lane_g_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    SetUpLane(&lane_h_, kLaneIdH, nullptr, &lane_g_, &segment_three_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_h_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{3. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_h_, DoToInertialPosition(Matches(kEndLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{4. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_h_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));
    // Segment four.
    SetUpLane(&lane_i_, kLaneIdI, nullptr, nullptr, &segment_four_, kLaneBounds, kLaneLength);
    EXPECT_CALL(lane_i_, DoToInertialPosition(Matches(kStartLanePositionPhase)))
        .WillRepeatedly(Return(api::InertialPosition{4. * kLaneLength, 0., 0.}));
    EXPECT_CALL(lane_i_, DoGetOrientation(_)).WillRepeatedly(Return(kRotation));

    EXPECT_CALL(id_index_, DoGetLane(kLaneIdA)).WillRepeatedly(Return(&lane_a_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdB)).WillRepeatedly(Return(&lane_b_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdC)).WillRepeatedly(Return(&lane_c_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdD)).WillRepeatedly(Return(&lane_d_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdE)).WillRepeatedly(Return(&lane_e_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdF)).WillRepeatedly(Return(&lane_f_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdG)).WillRepeatedly(Return(&lane_g_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdH)).WillRepeatedly(Return(&lane_h_));
    EXPECT_CALL(id_index_, DoGetLane(kLaneIdI)).WillRepeatedly(Return(&lane_i_));
    EXPECT_CALL(*(road_geometry_ptr_), DoById()).WillRepeatedly(ReturnRef(id_index_));

    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseZero{
        api::RoadPosition{&lane_a_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseZero{api::RoadPosition{&lane_a_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseOne{
        api::RoadPosition{&lane_b_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseOne{api::RoadPosition{&lane_d_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseTwo{
        api::RoadPosition{&lane_e_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseTwo{api::RoadPosition{&lane_e_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseThree{
        api::RoadPosition{&lane_f_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseThree{
        api::RoadPosition{&lane_h_, kEndLanePositionPhase}};
    const std::vector<api::RoadPosition> kStartRoadPositionsPhaseFour{
        api::RoadPosition{&lane_i_, kStartLanePositionPhase}};
    const std::vector<api::RoadPosition> kEndRoadPositionsPhaseFour{api::RoadPosition{&lane_i_, kEndLanePositionPhase}};
    phase_zero_ = std::make_unique<Phase>(kPhaseZeroIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseZero,
                                          kEndRoadPositionsPhaseZero, kLaneSRangesPhaseZero, road_network_.get());
    phase_one_ = std::make_unique<Phase>(kPhaseOneIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseOne,
                                         kEndRoadPositionsPhaseOne, kLaneSRangesPhaseOne, road_network_.get());
    phase_two_ = std::make_unique<Phase>(kPhaseTwoIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseTwo,
                                         kEndRoadPositionsPhaseTwo, kLaneSRangesPhaseTwo, road_network_.get());
    phase_three_ = std::make_unique<Phase>(kPhaseThreeIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseThree,
                                           kEndRoadPositionsPhaseThree, kLaneSRangesPhaseThree, road_network_.get());
    phase_four_ = std::make_unique<Phase>(kPhaseFourIndex, kLaneSRangeTolerance, kStartRoadPositionsPhaseFour,
                                          kEndRoadPositionsPhaseFour, kLaneSRangesPhaseFour, road_network_.get());
    EXPECT_CALL(*road_geometry_ptr_, do_linear_tolerance()).WillRepeatedly(Return(kLinearTolerance));
  }

 protected:
  LaneMock lane_e_;
  LaneMock lane_f_;
  LaneMock lane_g_;
  LaneMock lane_h_;
  LaneMock lane_i_;
  SegmentMock segment_two_;
  SegmentMock segment_three_;
  SegmentMock segment_four_;
  std::unique_ptr<Phase> phase_two_;
  std::unique_ptr<Phase> phase_three_;
  std::unique_ptr<Phase> phase_four_;
};

TEST_F(RouteComputeLaneSRouteCaseF2Test, ComputeLaneSRoute) {
  const api::RoadPosition kRoadPositionA{&lane_a_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionB{&lane_b_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionC{&lane_c_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionD{&lane_d_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionE{&lane_e_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionF{&lane_f_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionG{&lane_g_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionH{&lane_h_, kQueryLanePosition};
  const api::RoadPosition kRoadPositionI{&lane_i_, kQueryLanePosition};
  const api::LaneSRoute kExpectedLaneSRouteFromA({kLaneSRangeA, kLaneSRangeB, kLaneSRangeC, kLaneSRangeD, kLaneSRangeE,
                                                  kLaneSRangeF, kLaneSRangeG, kLaneSRangeH, kLaneSRangeI});
  const api::LaneSRoute kExpectedLaneSRouteFromB(
      {kLaneSRangeB, kLaneSRangeC, kLaneSRangeD, kLaneSRangeE, kLaneSRangeF, kLaneSRangeG, kLaneSRangeH, kLaneSRangeI});
  const api::LaneSRoute kExpectedLaneSRouteFromC(
      {kLaneSRangeC, kLaneSRangeD, kLaneSRangeE, kLaneSRangeF, kLaneSRangeG, kLaneSRangeH, kLaneSRangeI});
  const api::LaneSRoute kExpectedLaneSRouteFromD(
      {kLaneSRangeD, kLaneSRangeE, kLaneSRangeF, kLaneSRangeG, kLaneSRangeH, kLaneSRangeI});
  const api::LaneSRoute kExpectedLaneSRouteFromE(
      {kLaneSRangeE, kLaneSRangeF, kLaneSRangeG, kLaneSRangeH, kLaneSRangeI});
  const api::LaneSRoute kExpectedLaneSRouteFromF({kLaneSRangeF, kLaneSRangeG, kLaneSRangeH, kLaneSRangeI});
  const api::LaneSRoute kExpectedLaneSRouteFromG({kLaneSRangeG, kLaneSRangeH, kLaneSRangeI});
  const api::LaneSRoute kExpectedLaneSRouteFromH({kLaneSRangeH, kLaneSRangeI});
  const api::LaneSRoute kExpectedLaneSRouteFromI({kLaneSRangeI});
  const Route dut({*phase_zero_, *phase_one_, *phase_two_, *phase_three_, *phase_four_}, road_network_.get());

  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromA, dut.ComputeLaneSRoute(kRoadPositionA))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromB, dut.ComputeLaneSRoute(kRoadPositionB))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromC, dut.ComputeLaneSRoute(kRoadPositionC))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromD, dut.ComputeLaneSRoute(kRoadPositionD))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromE, dut.ComputeLaneSRoute(kRoadPositionE))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromF, dut.ComputeLaneSRoute(kRoadPositionF))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromG, dut.ComputeLaneSRoute(kRoadPositionG))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromH, dut.ComputeLaneSRoute(kRoadPositionH))));
  EXPECT_TRUE(AssertCompare(IsEqual(kExpectedLaneSRouteFromI, dut.ComputeLaneSRoute(kRoadPositionI))));
}

// Route::ValidateEndToEndConnectivity() is tested in maliput_integration_tests because it requires
// a fully implemented backend.

}  // namespace
}  // namespace test
}  // namespace routing
}  // namespace maliput
