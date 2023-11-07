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
#include "maliput/base/distance_router.h"

#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/road_network.h"
#include "maliput/common/assertion_error.h"
#include "maliput/routing/phase.h"
#include "maliput/routing/route.h"
#include "maliput/routing/routing_constraints.h"
#include "road_network_mocks.h"

namespace maliput {
namespace test {
namespace {

using maliput::test::IdIndexMock;
using maliput::test::LaneMock;
using maliput::test::RoadGeometryMock;
using ::testing::Return;
using ::testing::ReturnRef;
class DistanceRouterTest : public ::testing::Test {
 public:
  static constexpr double kLaneSRangeTolerance{1e-12};
};

TEST_F(DistanceRouterTest, NullptrRoadNetworkInConstructorThrows) {
  EXPECT_THROW({ DistanceRouter(nullptr, kLaneSRangeTolerance); }, common::assertion_error);
}

TEST_F(DistanceRouterTest, NegativeLaneSRangeToleranceInConstructorThrows) {
  auto road_network = MakeMockedRoadNetwork();

  EXPECT_THROW({ DistanceRouter(road_network.get(), -1.0); }, common::assertion_error);
}

TEST_F(DistanceRouterTest, CorrectConstruction) {
  auto road_network = MakeMockedRoadNetwork();

  EXPECT_NO_THROW({ DistanceRouter(road_network.get(), kLaneSRangeTolerance); });
}

class DistanceRouterComputeRoutesTest : public DistanceRouterTest {
 public:
  const api::LaneId kStartLaneId{"start_lane"};
  const api::LaneId kEndLaneId{"end_lane"};
  const routing::RoutingConstraints kConstraints{};

  void SetUp() override {
    road_network_ = MakeMockedRoadNetwork();
    RoadGeometryMock* road_geometry =
        static_cast<RoadGeometryMock*>(const_cast<api::RoadGeometry*>(road_network_->road_geometry()));
    EXPECT_CALL(*road_geometry, DoById()).WillRepeatedly(ReturnRef(id_index_));
    EXPECT_CALL(start_lane_, do_id()).WillRepeatedly(Return(kStartLaneId));
    EXPECT_CALL(end_lane_, do_id()).WillRepeatedly(Return(kEndLaneId));
  }

  std::unique_ptr<api::RoadNetwork> road_network_;
  LaneMock start_lane_;
  LaneMock end_lane_;
  IdIndexMock id_index_;
};

TEST_F(DistanceRouterComputeRoutesTest, InvalidStartLaneInComputeRoutesThrows) {
  EXPECT_CALL(id_index_, DoGetLane(kStartLaneId)).WillRepeatedly(Return(nullptr));
  EXPECT_CALL(id_index_, DoGetLane(kEndLaneId)).WillRepeatedly(Return(&end_lane_));
  const api::RoadPosition start_with_null_lane;
  const api::RoadPosition start_with_non_null_lane(&start_lane_, api::LanePosition(1., 2., 3.));
  const api::RoadPosition end(&end_lane_, api::LanePosition(4., 5., 6.));

  const DistanceRouter dut(road_network_.get(), kLaneSRangeTolerance);

  EXPECT_THROW({ dut.ComputeRoutes(start_with_null_lane, end, kConstraints); }, common::assertion_error);
  EXPECT_THROW({ dut.ComputeRoutes(start_with_non_null_lane, end, kConstraints); }, common::assertion_error);
}

TEST_F(DistanceRouterComputeRoutesTest, InvalidEndLaneInComputeRoutesThrows) {
  EXPECT_CALL(id_index_, DoGetLane(kStartLaneId)).WillRepeatedly(Return(&start_lane_));
  EXPECT_CALL(id_index_, DoGetLane(kEndLaneId)).WillRepeatedly(Return(nullptr));
  const api::RoadPosition start(&start_lane_, api::LanePosition(1., 2., 3.));
  const api::RoadPosition end_with_null_lane;
  const api::RoadPosition end_with_non_null_lane(&end_lane_, api::LanePosition(4., 5., 6.));

  const DistanceRouter dut(road_network_.get(), kLaneSRangeTolerance);

  EXPECT_THROW({ dut.ComputeRoutes(start, end_with_null_lane, kConstraints); }, common::assertion_error);
  EXPECT_THROW({ dut.ComputeRoutes(start, end_with_non_null_lane, kConstraints); }, common::assertion_error);
}

TEST_F(DistanceRouterComputeRoutesTest, InvalidConstraintsInComputeRoutesThrows) {
  EXPECT_CALL(id_index_, DoGetLane(kStartLaneId)).WillRepeatedly(Return(&start_lane_));
  EXPECT_CALL(id_index_, DoGetLane(kEndLaneId)).WillRepeatedly(Return(&end_lane_));
  const api::RoadPosition start(&start_lane_, api::LanePosition(1., 2., 3.));
  const api::RoadPosition end(&end_lane_, api::LanePosition(4., 5., 6.));
  const routing::RoutingConstraints kInvalidConstraints{true, {-1.}, {}};

  const DistanceRouter dut(road_network_.get(), kLaneSRangeTolerance);

  EXPECT_THROW({ dut.ComputeRoutes(start, end, kInvalidConstraints); }, common::assertion_error);
}

}  // namespace
}  // namespace test
}  // namespace maliput
