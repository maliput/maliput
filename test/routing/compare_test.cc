// BSD 3-Clause License
//
// Copyright (c) 2023-2026, Woven by Toyota. All rights reserved.
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
#include "maliput/routing/compare.h"

#include <gtest/gtest.h>

#include "assert_compare.h"
#include "maliput/api/lane_data.h"

namespace maliput {
namespace routing {
namespace test {
namespace {

static constexpr double kTolerance = 1e-3;

using maliput::test::AssertCompare;

TEST(IsPhasePositionResultClose, Test) {
  constexpr int kLaneSRangeIndex{1};
  constexpr int kDifferentLaneSRangeIndex{2};
  const api::LanePosition kLanePosition{1., 2., 3.};
  const api::LanePosition kDifferentLanePosition{4., 5., 6.};
  const api::InertialPosition kInertialPosition{7., 8., 9.};
  const api::InertialPosition kDifferentInertialPosition{10., 11., 12.};
  constexpr double kDistance{0.1};
  constexpr double kDifferentDistance{0.2};
  const PhasePositionResult ppr{kLaneSRangeIndex, kLanePosition, kInertialPosition, kDistance};
  const PhasePositionResult ppr_different_lane_s_range_index{kDifferentLaneSRangeIndex, kLanePosition,
                                                             kInertialPosition, kDistance};
  const PhasePositionResult ppr_different_lane_position{kLaneSRangeIndex, kDifferentLanePosition, kInertialPosition,
                                                        kDistance};
  const PhasePositionResult ppr_different_inertial_position{kLaneSRangeIndex, kLanePosition, kDifferentInertialPosition,
                                                            kDistance};
  const PhasePositionResult ppr_different_distance{kLaneSRangeIndex, kLanePosition, kInertialPosition,
                                                   kDifferentDistance};

  EXPECT_EQ(testing::AssertionFailure(),
            AssertCompare(IsPhasePositionResultClose(ppr, ppr_different_lane_s_range_index, kTolerance)));
  EXPECT_EQ(testing::AssertionFailure(),
            AssertCompare(IsPhasePositionResultClose(ppr, ppr_different_lane_position, kTolerance)));
  EXPECT_EQ(testing::AssertionFailure(),
            AssertCompare(IsPhasePositionResultClose(ppr, ppr_different_inertial_position, kTolerance)));
  EXPECT_EQ(testing::AssertionFailure(),
            AssertCompare(IsPhasePositionResultClose(ppr, ppr_different_distance, kTolerance)));
  EXPECT_EQ(testing::AssertionSuccess(), AssertCompare(IsPhasePositionResultClose(ppr, ppr, kTolerance)));
}

TEST(IsRoutePositionResultClose, Test) {
  constexpr int kPhaseIndex{0};
  constexpr int kDifferentPhaseIndex{1};
  constexpr int kLaneSRangeIndex{2};
  constexpr int kDifferentLaneSRangeIndex{3};
  const api::LanePosition kLanePosition{1., 2., 3.};
  const api::InertialPosition kInertialPosition{7., 8., 9.};
  constexpr double kDistance{0.1};
  const PhasePositionResult ppr{kLaneSRangeIndex, kLanePosition, kInertialPosition, kDistance};
  const PhasePositionResult ppr_different_lane_s_range_index{kDifferentLaneSRangeIndex, kLanePosition,
                                                             kInertialPosition, kDistance};
  const RoutePositionResult rpr{kPhaseIndex, ppr};
  const RoutePositionResult rpr_different_phase_index{kDifferentPhaseIndex, ppr};
  const RoutePositionResult rpr_different_ppr{kPhaseIndex, ppr_different_lane_s_range_index};

  EXPECT_EQ(testing::AssertionFailure(),
            AssertCompare(IsRoutePositionResultClose(rpr, rpr_different_phase_index, kTolerance)));
  EXPECT_EQ(testing::AssertionFailure(), AssertCompare(IsRoutePositionResultClose(rpr, rpr_different_ppr, kTolerance)));
  EXPECT_EQ(testing::AssertionSuccess(), AssertCompare(IsRoutePositionResultClose(rpr, rpr, kTolerance)));
}

}  // namespace
}  // namespace test
}  // namespace routing
}  // namespace maliput
