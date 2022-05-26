// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
#include <gtest/gtest.h>

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace api {
namespace {

// Tests the streaming string operators for the following Maliput abstractions:
//   - InertialPosition
//   - LaneEnd::Which
//   - LanePosition
//   - Rotation
GTEST_TEST(MaliputApiTest, TestLaneDataToStringStream) {
  std::stringstream buffer;

  // Tests InertialPosition.
  buffer << InertialPosition(1.5, 2.2, 3.7);
  EXPECT_EQ(buffer.str(), "(x = 1.5, y = 2.2, z = 3.7)");
  buffer.str("");
  buffer.clear();

  // Tests LaneEnd::Which.
  LaneEnd::Which start = LaneEnd::kStart;
  buffer << start;
  EXPECT_EQ(buffer.str(), "start");
  buffer.str("");
  buffer.clear();
  LaneEnd::Which end = LaneEnd::kFinish;
  buffer << end;
  EXPECT_EQ(buffer.str(), "finish");
  buffer.str("");
  buffer.clear();

  // Tests LanePosition.
  buffer << LanePosition(0.2, 9.1, 15.2);
  EXPECT_EQ(buffer.str(), "(s = 0.2, r = 9.1, h = 15.2)");
  buffer.str("");
  buffer.clear();

  // Tests Rotation.
  buffer << Rotation::FromRpy(M_PI / 5., M_PI / 6., M_PI / 7.);
  EXPECT_EQ(buffer.str(), "(roll = 0.628319, pitch = 0.523599, yaw = 0.448799)");
  buffer.str("");
  buffer.clear();
}

}  // namespace
}  // namespace api
}  // namespace maliput
