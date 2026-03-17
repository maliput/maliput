// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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
#include "maliput/base/road_object_book.h"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/api/objects/road_object.h"
#include "maliput/math/bounding_box.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace {

using api::InertialPosition;
using api::LaneId;
using api::LanePosition;
using api::Rotation;
using api::objects::Outline;
using api::objects::OutlineCorner;
using api::objects::RoadObject;
using api::objects::RoadObjectPosition;
using api::objects::RoadObjectType;

// Concrete subclass of RoadObject for testing.
class TestRoadObject final : public RoadObject {
 public:
  TestRoadObject(const Id& id, RoadObjectType type, const RoadObjectPosition& position, const Rotation& orientation,
                 const math::BoundingBox& bounding_box, bool is_dynamic, std::vector<LaneId> related_lanes,
                 std::optional<std::string> name, std::optional<std::string> subtype,
                 std::vector<std::unique_ptr<Outline>> outlines,
                 std::unordered_map<std::string, std::string> properties)
      : RoadObject(id, type, position, orientation, bounding_box, is_dynamic, std::move(related_lanes), std::move(name),
                   std::move(subtype), std::move(outlines), std::move(properties)) {}
};

// Helper to create a RoadObject with a given ID, type, position, and related lanes.
std::unique_ptr<TestRoadObject> MakeRoadObject(const std::string& id, RoadObjectType type, double x, double y, double z,
                                               std::vector<LaneId> related_lanes = {}) {
  const RoadObjectPosition position(InertialPosition(x, y, z));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const math::BoundingBox bounding_box(math::Vector3(x, y, z), math::Vector3(1., 1., 1.),
                                       math::RollPitchYaw(0., 0., 0.), 0.01);
  return std::unique_ptr<TestRoadObject>(new TestRoadObject(RoadObject::Id(id), type, position, orientation,
                                                            bounding_box, false, std::move(related_lanes), std::nullopt,
                                                            std::nullopt, {}, {}));
}

// -- Empty book tests --

GTEST_TEST(RoadObjectBookTest, EmptyBook) {
  RoadObjectBook dut;

  EXPECT_TRUE(dut.RoadObjects().empty());
  EXPECT_EQ(dut.GetRoadObject(RoadObject::Id("nonexistent")), nullptr);
  EXPECT_TRUE(dut.FindByType(RoadObjectType::kBarrier).empty());
  EXPECT_TRUE(dut.FindByLane(LaneId("lane_1")).empty());
  EXPECT_TRUE(dut.FindInRadius(InertialPosition(0., 0., 0.), 100.).empty());
}

// -- AddRoadObject and GetRoadObject --

GTEST_TEST(RoadObjectBookTest, AddAndGet) {
  RoadObjectBook dut;

  auto obj = MakeRoadObject("obj_1", RoadObjectType::kBarrier, 10., 20., 0.);
  const RoadObject* obj_ptr = obj.get();
  dut.AddRoadObject(std::move(obj));

  const RoadObject* result = dut.GetRoadObject(RoadObject::Id("obj_1"));
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result, obj_ptr);
  EXPECT_EQ(result->id().string(), "obj_1");
  EXPECT_EQ(result->type(), RoadObjectType::kBarrier);
}

GTEST_TEST(RoadObjectBookTest, GetNonexistent) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("obj_1", RoadObjectType::kBarrier, 0., 0., 0.));
  EXPECT_EQ(dut.GetRoadObject(RoadObject::Id("obj_2")), nullptr);
}

GTEST_TEST(RoadObjectBookTest, DuplicateIdThrows) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("obj_1", RoadObjectType::kBarrier, 0., 0., 0.));
  EXPECT_THROW(dut.AddRoadObject(MakeRoadObject("obj_1", RoadObjectType::kPole, 1., 1., 1.)), std::logic_error);
}

GTEST_TEST(RoadObjectBookTest, NullptrThrows) {
  RoadObjectBook dut;

  EXPECT_THROW(dut.AddRoadObject(nullptr), std::exception);
}

// -- RoadObjects (list all) --

GTEST_TEST(RoadObjectBookTest, RoadObjectsList) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("obj_1", RoadObjectType::kBarrier, 0., 0., 0.));
  dut.AddRoadObject(MakeRoadObject("obj_2", RoadObjectType::kTree, 1., 1., 1.));
  dut.AddRoadObject(MakeRoadObject("obj_3", RoadObjectType::kPole, 2., 2., 2.));

  const auto result = dut.RoadObjects();
  EXPECT_EQ(static_cast<int>(result.size()), 3);
}

// -- FindByType --

GTEST_TEST(RoadObjectBookTest, FindByType) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("barrier_1", RoadObjectType::kBarrier, 0., 0., 0.));
  dut.AddRoadObject(MakeRoadObject("barrier_2", RoadObjectType::kBarrier, 1., 0., 0.));
  dut.AddRoadObject(MakeRoadObject("tree_1", RoadObjectType::kTree, 2., 0., 0.));

  const auto barriers = dut.FindByType(RoadObjectType::kBarrier);
  EXPECT_EQ(static_cast<int>(barriers.size()), 2);

  const auto trees = dut.FindByType(RoadObjectType::kTree);
  EXPECT_EQ(static_cast<int>(trees.size()), 1);
  EXPECT_EQ(trees[0]->id().string(), "tree_1");

  const auto poles = dut.FindByType(RoadObjectType::kPole);
  EXPECT_TRUE(poles.empty());
}

// -- FindByLane --

GTEST_TEST(RoadObjectBookTest, FindByLane) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("obj_1", RoadObjectType::kBarrier, 0., 0., 0., {LaneId("lane_a")}));
  dut.AddRoadObject(
      MakeRoadObject("obj_2", RoadObjectType::kCrosswalk, 1., 0., 0., {LaneId("lane_a"), LaneId("lane_b")}));
  dut.AddRoadObject(MakeRoadObject("obj_3", RoadObjectType::kTree, 2., 0., 0., {LaneId("lane_c")}));

  const auto lane_a_objs = dut.FindByLane(LaneId("lane_a"));
  EXPECT_EQ(static_cast<int>(lane_a_objs.size()), 2);

  const auto lane_b_objs = dut.FindByLane(LaneId("lane_b"));
  EXPECT_EQ(static_cast<int>(lane_b_objs.size()), 1);
  EXPECT_EQ(lane_b_objs[0]->id().string(), "obj_2");

  const auto lane_c_objs = dut.FindByLane(LaneId("lane_c"));
  EXPECT_EQ(static_cast<int>(lane_c_objs.size()), 1);
  EXPECT_EQ(lane_c_objs[0]->id().string(), "obj_3");

  const auto lane_d_objs = dut.FindByLane(LaneId("lane_d"));
  EXPECT_TRUE(lane_d_objs.empty());
}

// -- FindInRadius --

GTEST_TEST(RoadObjectBookTest, FindInRadiusAllWithin) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("obj_1", RoadObjectType::kPole, 0., 0., 0.));
  dut.AddRoadObject(MakeRoadObject("obj_2", RoadObjectType::kPole, 1., 0., 0.));

  // Large radius catches both objects.
  const auto result = dut.FindInRadius(InertialPosition(0., 0., 0.), 10.);
  EXPECT_EQ(static_cast<int>(result.size()), 2);
}

GTEST_TEST(RoadObjectBookTest, FindInRadiusPartial) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("near", RoadObjectType::kPole, 1., 0., 0.));
  dut.AddRoadObject(MakeRoadObject("far", RoadObjectType::kPole, 100., 0., 0.));

  // Radius of 5 catches only the near object.
  const auto result = dut.FindInRadius(InertialPosition(0., 0., 0.), 5.);
  EXPECT_EQ(static_cast<int>(result.size()), 1);
  EXPECT_EQ(result[0]->id().string(), "near");
}

GTEST_TEST(RoadObjectBookTest, FindInRadiusNoneWithin) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("far_1", RoadObjectType::kBarrier, 100., 100., 100.));

  const auto result = dut.FindInRadius(InertialPosition(0., 0., 0.), 1.);
  EXPECT_TRUE(result.empty());
}

GTEST_TEST(RoadObjectBookTest, FindInRadiusZeroRadius) {
  RoadObjectBook dut;

  dut.AddRoadObject(MakeRoadObject("at_origin", RoadObjectType::kPole, 0., 0., 0.));
  dut.AddRoadObject(MakeRoadObject("not_at_origin", RoadObjectType::kPole, 1., 0., 0.));

  // Zero radius: only exact match.
  const auto result = dut.FindInRadius(InertialPosition(0., 0., 0.), 0.);
  EXPECT_EQ(static_cast<int>(result.size()), 1);
  EXPECT_EQ(result[0]->id().string(), "at_origin");
}

GTEST_TEST(RoadObjectBookTest, FindInRadius3D) {
  RoadObjectBook dut;

  // Object at (0, 0, 0). Distance to (3, 4, 0) = 5.
  dut.AddRoadObject(MakeRoadObject("obj_1", RoadObjectType::kPole, 0., 0., 0.));

  // Radius of exactly 5 should include it (<=).
  const auto within = dut.FindInRadius(InertialPosition(3., 4., 0.), 5.);
  EXPECT_EQ(static_cast<int>(within.size()), 1);

  // Radius of 4.99 should exclude it.
  const auto outside = dut.FindInRadius(InertialPosition(3., 4., 0.), 4.99);
  EXPECT_TRUE(outside.empty());
}

GTEST_TEST(RoadObjectBookTest, FindInRadiusNegativeRadiusThrows) {
  RoadObjectBook dut;

  EXPECT_THROW(dut.FindInRadius(InertialPosition(0., 0., 0.), -1.), std::exception);
}

}  // namespace
}  // namespace maliput
