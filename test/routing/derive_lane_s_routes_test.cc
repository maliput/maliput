// BSD 3-Clause License
//
// Copyright (c) 2024, Woven by Toyota. All rights reserved.
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
#include "maliput/routing/derive_lane_s_routes.h"

#include <optional>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/common/maliput_error.h"
#include "road_network_mocks.h"

namespace maliput {
namespace routing {
namespace test {
namespace {

using maliput::test::LaneEndSetMock;
using maliput::test::LaneMock;
using ::testing::Return;
using ::testing::ReturnRef;

class DetermineEdgeSTest : public ::testing::Test {
 public:
  void SetUp() override {
    EXPECT_CALL(lane_, DoGetOngoingBranches(api::LaneEnd::kFinish)).WillRepeatedly(Return(&finish_lane_end_set_));
    EXPECT_CALL(lane_, DoGetOngoingBranches(api::LaneEnd::kStart)).WillRepeatedly(Return(&start_lane_end_set_));
  }

  LaneMock lane_;
  LaneMock next_lane_;
  LaneEndSetMock finish_lane_end_set_;
  LaneEndSetMock start_lane_end_set_;
};

TEST_F(DetermineEdgeSTest, NextLaneIsInOngoingBranchesAtFinishReturnsLaneLength) {
  constexpr double kLaneLength{100.};
  const api::LaneEnd lane_end(&next_lane_, api::LaneEnd::kStart);
  EXPECT_CALL(lane_, do_length()).WillRepeatedly(Return(kLaneLength));
  EXPECT_CALL(finish_lane_end_set_, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(finish_lane_end_set_, do_get(0)).WillRepeatedly(ReturnRef(lane_end));

  std::optional<double> result = DetermineEdgeS(lane_, next_lane_);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(kLaneLength, result.value());
}

TEST_F(DetermineEdgeSTest, NextLaneIsInOngoingBranchesAtStartReturnsZero) {
  const api::LaneEnd lane_end(&next_lane_, api::LaneEnd::kStart);
  EXPECT_CALL(finish_lane_end_set_, do_size()).WillRepeatedly(Return(0));
  EXPECT_CALL(start_lane_end_set_, do_size()).WillRepeatedly(Return(1));
  EXPECT_CALL(start_lane_end_set_, do_get(0)).WillRepeatedly(ReturnRef(lane_end));

  std::optional<double> result = DetermineEdgeS(lane_, next_lane_);

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(0., result.value());
}

TEST_F(DetermineEdgeSTest, NextLaneIsNotInOngoingBranchesReturnsNullopt) {
  EXPECT_CALL(finish_lane_end_set_, do_size()).WillRepeatedly(Return(0));
  EXPECT_CALL(start_lane_end_set_, do_size()).WillRepeatedly(Return(0));

  std::optional<double> result = DetermineEdgeS(lane_, next_lane_);

  ASSERT_FALSE(result.has_value());
}

}  // namespace
}  // namespace test
}  // namespace routing
}  // namespace maliput
