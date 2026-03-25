// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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
#include "maliput/base/traffic_sign_book.h"

#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/rules/traffic_sign.h"
#include "maliput/common/error.h"
#include "maliput/math/bounding_box.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace {

using api::LaneId;
using api::rules::TrafficSign;
using api::rules::TrafficSignType;

/// Returns a default bounding box suitable for use in tests.
maliput::math::BoundingBox MakeBoundingBox() {
  return maliput::math::BoundingBox{maliput::math::Vector3(0., 0., 0.), maliput::math::Vector3(0.01, 0.75, 0.75),
                                    maliput::math::RollPitchYaw(0., 0., 0.), 1e-3};
}

GTEST_TEST(TrafficSignBookTest, BasicTest) {
  const TrafficSign::Id id("my_stop_sign");
  const std::vector<LaneId> related_lanes{LaneId("lane_1"), LaneId("lane_2")};
  auto sign = std::make_unique<const TrafficSign>(id, TrafficSignType::kStop, api::InertialPosition(1., 2., 3.),
                                                  api::Rotation::FromRpy(0., 0., 0.),
                                                  std::optional<std::string>{"STOP"}, related_lanes, MakeBoundingBox());
  const TrafficSign* sign_ptr = sign.get();

  TrafficSignBook dut;

  const std::vector<const TrafficSign*> empty = dut.TrafficSigns();
  EXPECT_TRUE(empty.empty());

  dut.AddTrafficSign(std::move(sign));

  EXPECT_EQ(dut.GetTrafficSign(TrafficSign::Id("unknown_sign")), nullptr);
  EXPECT_EQ(dut.GetTrafficSign(id), sign_ptr);

  const std::vector<const TrafficSign*> nonempty = dut.TrafficSigns();
  ASSERT_EQ(static_cast<int>(nonempty.size()), 1);
  EXPECT_EQ(nonempty.at(0), sign_ptr);
}

GTEST_TEST(TrafficSignBookTest, DuplicateIdThrows) {
  const TrafficSign::Id id("dup_sign");
  auto sign_a = std::make_unique<const TrafficSign>(id, TrafficSignType::kYield, api::InertialPosition(0., 0., 0.),
                                                    api::Rotation::FromRpy(0., 0., 0.), std::nullopt,
                                                    std::vector<LaneId>{}, MakeBoundingBox());
  auto sign_b = std::make_unique<const TrafficSign>(id, TrafficSignType::kYield, api::InertialPosition(1., 1., 1.),
                                                    api::Rotation::FromRpy(0., 0., 0.), std::nullopt,
                                                    std::vector<LaneId>{}, MakeBoundingBox());

  TrafficSignBook dut;
  dut.AddTrafficSign(std::move(sign_a));
  EXPECT_THROW(dut.AddTrafficSign(std::move(sign_b)), maliput::common::traffic_sign_book_error);
}

GTEST_TEST(TrafficSignBookTest, FindByLane) {
  const TrafficSign::Id id_a("sign_a");
  const TrafficSign::Id id_b("sign_b");
  const LaneId lane_1("lane_1");
  const LaneId lane_2("lane_2");
  const LaneId lane_unknown("lane_unknown");

  auto sign_a = std::make_unique<const TrafficSign>(id_a, TrafficSignType::kStop, api::InertialPosition(0., 0., 0.),
                                                    api::Rotation::FromRpy(0., 0., 0.), std::nullopt,
                                                    std::vector<LaneId>{lane_1, lane_2}, MakeBoundingBox());
  auto sign_b = std::make_unique<const TrafficSign>(id_b, TrafficSignType::kYield, api::InertialPosition(1., 1., 1.),
                                                    api::Rotation::FromRpy(0., 0., 0.), std::nullopt,
                                                    std::vector<LaneId>{lane_2}, MakeBoundingBox());

  TrafficSignBook dut;
  dut.AddTrafficSign(std::move(sign_a));
  dut.AddTrafficSign(std::move(sign_b));

  // lane_1 is related only to sign_a.
  const auto result_lane_1 = dut.FindByLane(lane_1);
  ASSERT_EQ(static_cast<int>(result_lane_1.size()), 1);
  EXPECT_EQ(result_lane_1[0]->id(), id_a);

  // lane_2 is related to both sign_a and sign_b.
  const auto result_lane_2 = dut.FindByLane(lane_2);
  EXPECT_EQ(static_cast<int>(result_lane_2.size()), 2);

  // Unknown lane returns empty.
  const auto result_unknown = dut.FindByLane(lane_unknown);
  EXPECT_TRUE(result_unknown.empty());
}

GTEST_TEST(TrafficSignBookTest, FindByType) {
  const TrafficSign::Id id_stop("stop_sign");
  const TrafficSign::Id id_yield("yield_sign");
  const TrafficSign::Id id_speed("speed_sign");

  auto stop = std::make_unique<const TrafficSign>(id_stop, TrafficSignType::kStop, api::InertialPosition(0., 0., 0.),
                                                  api::Rotation::FromRpy(0., 0., 0.), std::nullopt,
                                                  std::vector<LaneId>{}, MakeBoundingBox());
  auto yield = std::make_unique<const TrafficSign>(id_yield, TrafficSignType::kYield, api::InertialPosition(1., 1., 1.),
                                                   api::Rotation::FromRpy(0., 0., 0.), std::nullopt,
                                                   std::vector<LaneId>{}, MakeBoundingBox());
  auto speed = std::make_unique<const TrafficSign>(
      id_speed, TrafficSignType::kSpeedLimit, api::InertialPosition(2., 2., 2.), api::Rotation::FromRpy(0., 0., 0.),
      std::optional<std::string>{"60"}, std::vector<LaneId>{}, MakeBoundingBox());

  TrafficSignBook dut;
  dut.AddTrafficSign(std::move(stop));
  dut.AddTrafficSign(std::move(yield));
  dut.AddTrafficSign(std::move(speed));

  // Exactly one stop sign.
  const auto stops = dut.FindByType(TrafficSignType::kStop);
  ASSERT_EQ(static_cast<int>(stops.size()), 1);
  EXPECT_EQ(stops[0]->id(), id_stop);

  // Exactly one yield sign.
  const auto yields = dut.FindByType(TrafficSignType::kYield);
  ASSERT_EQ(static_cast<int>(yields.size()), 1);
  EXPECT_EQ(yields[0]->id(), id_yield);

  // No-entry signs are absent.
  const auto no_entry = dut.FindByType(TrafficSignType::kNoEntry);
  EXPECT_TRUE(no_entry.empty());
}

}  // namespace
}  // namespace maliput
