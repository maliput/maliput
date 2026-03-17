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
#include "maliput/api/objects/road_object.h"

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/common/error.h"
#include "maliput/math/bounding_box.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace api {
namespace objects {
namespace {

// Concrete subclass of RoadObject to expose the protected constructor for
// testing purposes.
class TestRoadObject final : public RoadObject {
 public:
  TestRoadObject(const Id& id, RoadObjectType type, const RoadObjectPosition& position, const Rotation& orientation,
                 const maliput::math::BoundingBox& bounding_box, bool is_dynamic, std::vector<LaneId> related_lanes,
                 std::optional<std::string> name, std::optional<std::string> subtype,
                 std::vector<std::unique_ptr<Outline>> outlines,
                 std::unordered_map<std::string, std::string> properties)
      : RoadObject(id, type, position, orientation, bounding_box, is_dynamic, std::move(related_lanes), std::move(name),
                   std::move(subtype), std::move(outlines), std::move(properties)) {}
};

// -- RoadObjectType tests --

GTEST_TEST(RoadObjectTypeTest, InstantiateAndAssign) {
  RoadObjectType dut{};
  EXPECT_EQ(dut, RoadObjectType::kUnknown);
  for (RoadObjectType type : {RoadObjectType::kBarrier, RoadObjectType::kBuilding, RoadObjectType::kCrosswalk,
                              RoadObjectType::kGantry, RoadObjectType::kObstacle, RoadObjectType::kParkingSpace,
                              RoadObjectType::kPole, RoadObjectType::kRoadMark, RoadObjectType::kRoadSurface,
                              RoadObjectType::kTrafficIsland, RoadObjectType::kTree, RoadObjectType::kVegetation}) {
    dut = type;
    EXPECT_EQ(dut, type);
  }
}

GTEST_TEST(RoadObjectTypeTest, MapperTest) {
  const auto dut = RoadObjectTypeMapper();
  const std::vector<RoadObjectType> expected_types{
      RoadObjectType::kUnknown,      RoadObjectType::kBarrier,       RoadObjectType::kBuilding,
      RoadObjectType::kCrosswalk,    RoadObjectType::kGantry,        RoadObjectType::kObstacle,
      RoadObjectType::kParkingSpace, RoadObjectType::kPole,          RoadObjectType::kRoadMark,
      RoadObjectType::kRoadSurface,  RoadObjectType::kTrafficIsland, RoadObjectType::kTree,
      RoadObjectType::kVegetation,
  };
  EXPECT_EQ(dut.size(), expected_types.size());
  for (RoadObjectType type : expected_types) {
    EXPECT_EQ(static_cast<int>(dut.count(type)), 1);
  }
}

GTEST_TEST(RoadObjectTypeTest, HashWorks) {
  const std::hash<RoadObjectType> hasher;
  // Just verify it doesn't crash and returns values.
  EXPECT_NO_THROW(hasher(RoadObjectType::kBarrier));
  EXPECT_NO_THROW(hasher(RoadObjectType::kUnknown));
}

// -- OutlineCorner tests --

GTEST_TEST(OutlineCornerTest, ConstructionWithoutHeight) {
  const math::Vector3 position(1., 2., 3.);
  const OutlineCorner dut(position);
  EXPECT_EQ(dut.position().x(), 1.);
  EXPECT_EQ(dut.position().y(), 2.);
  EXPECT_EQ(dut.position().z(), 3.);
  EXPECT_FALSE(dut.height().has_value());
}

GTEST_TEST(OutlineCornerTest, ConstructionWithHeight) {
  const math::Vector3 position(4., 5., 6.);
  const OutlineCorner dut(position, 2.5);
  EXPECT_EQ(dut.position().x(), 4.);
  EXPECT_EQ(dut.position().y(), 5.);
  EXPECT_EQ(dut.position().z(), 6.);
  ASSERT_TRUE(dut.height().has_value());
  EXPECT_DOUBLE_EQ(dut.height().value(), 2.5);
}

GTEST_TEST(OutlineCornerTest, CopyConstructor) {
  const OutlineCorner original(math::Vector3(1., 2., 3.), 4.);
  const OutlineCorner copy(original);
  EXPECT_EQ(copy.position().x(), original.position().x());
  EXPECT_EQ(copy.position().y(), original.position().y());
  EXPECT_EQ(copy.position().z(), original.position().z());
  EXPECT_EQ(copy.height(), original.height());
}

// -- Outline tests --

GTEST_TEST(OutlineTest, ValidConstruction) {
  const Outline::Id id("outline_1");
  std::vector<OutlineCorner> corners;
  corners.emplace_back(math::Vector3(0., 0., 0.));
  corners.emplace_back(math::Vector3(1., 0., 0.));
  corners.emplace_back(math::Vector3(1., 1., 0.));
  const Outline dut(id, std::move(corners), true);

  EXPECT_EQ(dut.id().string(), "outline_1");
  EXPECT_EQ(dut.num_corners(), 3);
  EXPECT_TRUE(dut.is_closed());
  EXPECT_EQ(dut.corners().size(), 3u);
}

GTEST_TEST(OutlineTest, OpenOutline) {
  const Outline::Id id("open_outline");
  std::vector<OutlineCorner> corners;
  corners.emplace_back(math::Vector3(0., 0., 0.));
  corners.emplace_back(math::Vector3(1., 0., 0.));
  corners.emplace_back(math::Vector3(2., 0., 0.));
  const Outline dut(id, std::move(corners), false);

  EXPECT_FALSE(dut.is_closed());
}

GTEST_TEST(OutlineTest, DefaultClosedParameter) {
  const Outline::Id id("default_closed");
  std::vector<OutlineCorner> corners;
  corners.emplace_back(math::Vector3(0., 0., 0.));
  corners.emplace_back(math::Vector3(1., 0., 0.));
  corners.emplace_back(math::Vector3(1., 1., 0.));
  const Outline dut(id, std::move(corners));

  EXPECT_TRUE(dut.is_closed());
}

GTEST_TEST(OutlineTest, TooFewCorners) {
  const Outline::Id id("bad_outline");
  std::vector<OutlineCorner> corners;
  corners.emplace_back(math::Vector3(0., 0., 0.));
  corners.emplace_back(math::Vector3(1., 0., 0.));
  EXPECT_THROW(Outline(id, std::move(corners), true), maliput::common::assertion_error);
}

GTEST_TEST(OutlineTest, CornersWithHeight) {
  const Outline::Id id("height_outline");
  std::vector<OutlineCorner> corners;
  corners.emplace_back(math::Vector3(0., 0., 0.), 1.0);
  corners.emplace_back(math::Vector3(1., 0., 0.), 2.0);
  corners.emplace_back(math::Vector3(1., 1., 0.), 1.5);
  const Outline dut(id, std::move(corners), true);

  EXPECT_EQ(dut.num_corners(), 3);
  ASSERT_TRUE(dut.corners()[0].height().has_value());
  EXPECT_DOUBLE_EQ(dut.corners()[0].height().value(), 1.0);
  ASSERT_TRUE(dut.corners()[1].height().has_value());
  EXPECT_DOUBLE_EQ(dut.corners()[1].height().value(), 2.0);
  ASSERT_TRUE(dut.corners()[2].height().has_value());
  EXPECT_DOUBLE_EQ(dut.corners()[2].height().value(), 1.5);
}

// -- RoadObjectPosition tests --

GTEST_TEST(RoadObjectPositionTest, InertialOnly) {
  const InertialPosition inertial_pos(10., 20., 30.);
  const RoadObjectPosition dut(inertial_pos);

  EXPECT_DOUBLE_EQ(dut.inertial_position().x(), 10.);
  EXPECT_DOUBLE_EQ(dut.inertial_position().y(), 20.);
  EXPECT_DOUBLE_EQ(dut.inertial_position().z(), 30.);
  EXPECT_FALSE(dut.has_lane_position());
  EXPECT_FALSE(dut.lane_id().has_value());
  EXPECT_FALSE(dut.lane_position().has_value());
}

GTEST_TEST(RoadObjectPositionTest, WithLaneAssociation) {
  const InertialPosition inertial_pos(10., 20., 30.);
  const LaneId lane_id("lane_1");
  const LanePosition lane_pos(5., 0.5, 0.);
  const RoadObjectPosition dut(inertial_pos, lane_id, lane_pos);

  EXPECT_DOUBLE_EQ(dut.inertial_position().x(), 10.);
  EXPECT_DOUBLE_EQ(dut.inertial_position().y(), 20.);
  EXPECT_DOUBLE_EQ(dut.inertial_position().z(), 30.);
  EXPECT_TRUE(dut.has_lane_position());
  ASSERT_TRUE(dut.lane_id().has_value());
  EXPECT_EQ(dut.lane_id()->string(), "lane_1");
  ASSERT_TRUE(dut.lane_position().has_value());
  EXPECT_DOUBLE_EQ(dut.lane_position()->s(), 5.);
  EXPECT_DOUBLE_EQ(dut.lane_position()->r(), 0.5);
  EXPECT_DOUBLE_EQ(dut.lane_position()->h(), 0.);
}

GTEST_TEST(RoadObjectPositionTest, CopyAssignment) {
  const InertialPosition inertial_pos(10., 20., 30.);
  const LaneId lane_id("lane_1");
  const LanePosition lane_pos(5., 0.5, 0.);
  const RoadObjectPosition original(inertial_pos, lane_id, lane_pos);
  RoadObjectPosition copy(InertialPosition(0., 0., 0.));
  copy = original;

  EXPECT_DOUBLE_EQ(copy.inertial_position().x(), 10.);
  EXPECT_TRUE(copy.has_lane_position());
  ASSERT_TRUE(copy.lane_id().has_value());
  EXPECT_EQ(copy.lane_id()->string(), "lane_1");
}

// -- RoadObject tests --

class RoadObjectTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const RoadObject::Id id("object_1");
    const RoadObjectType type = RoadObjectType::kBarrier;
    const RoadObjectPosition position(InertialPosition(1., 2., 3.));
    const Rotation orientation = Rotation::FromRpy(0., 0., 1.57);
    const maliput::math::BoundingBox bounding_box(math::Vector3(1., 2., 3.), math::Vector3(2., 1., 0.5),
                                                  math::RollPitchYaw(0., 0., 1.57), 0.01);
    const bool is_dynamic = false;
    std::vector<LaneId> related_lanes{LaneId("lane_a"), LaneId("lane_b")};
    std::optional<std::string> name("My Barrier");
    std::optional<std::string> subtype("guardRail");

    std::vector<std::unique_ptr<Outline>> outlines;
    std::vector<OutlineCorner> corners;
    corners.emplace_back(math::Vector3(0., 0., 0.), 1.0);
    corners.emplace_back(math::Vector3(1., 0., 0.), 1.0);
    corners.emplace_back(math::Vector3(1., 1., 0.), 1.0);
    outlines.push_back(std::make_unique<Outline>(Outline::Id("outline_a"), std::move(corners), true));

    std::unordered_map<std::string, std::string> properties{{"material", "steel"}, {"source_id", "obj_42"}};

    dut_ = std::unique_ptr<TestRoadObject>(
        new TestRoadObject(id, type, position, orientation, bounding_box, is_dynamic, std::move(related_lanes),
                           std::move(name), std::move(subtype), std::move(outlines), std::move(properties)));
  }

  std::unique_ptr<TestRoadObject> dut_;
};

TEST_F(RoadObjectTest, Accessors) {
  EXPECT_EQ(dut_->id().string(), "object_1");
  ASSERT_TRUE(dut_->name().has_value());
  EXPECT_EQ(dut_->name().value(), "My Barrier");
  EXPECT_EQ(dut_->type(), RoadObjectType::kBarrier);
  ASSERT_TRUE(dut_->subtype().has_value());
  EXPECT_EQ(dut_->subtype().value(), "guardRail");
  EXPECT_DOUBLE_EQ(dut_->position().inertial_position().x(), 1.);
  EXPECT_DOUBLE_EQ(dut_->position().inertial_position().y(), 2.);
  EXPECT_DOUBLE_EQ(dut_->position().inertial_position().z(), 3.);
  EXPECT_FALSE(dut_->is_dynamic());
}

TEST_F(RoadObjectTest, RelatedLanes) {
  const auto& lanes = dut_->related_lanes();
  ASSERT_EQ(static_cast<int>(lanes.size()), 2);
  EXPECT_EQ(lanes[0].string(), "lane_a");
  EXPECT_EQ(lanes[1].string(), "lane_b");
}

TEST_F(RoadObjectTest, Outlines) {
  EXPECT_EQ(dut_->num_outlines(), 1);
  const auto& outlines = dut_->outlines();
  ASSERT_EQ(static_cast<int>(outlines.size()), 1);
  EXPECT_EQ(outlines[0]->id().string(), "outline_a");
  EXPECT_EQ(outlines[0]->num_corners(), 3);
}

TEST_F(RoadObjectTest, OutlineByIndex) {
  const Outline* outline = dut_->outline(0);
  ASSERT_NE(outline, nullptr);
  EXPECT_EQ(outline->id().string(), "outline_a");
}

TEST_F(RoadObjectTest, OutlineOutOfRange) {
  EXPECT_THROW(dut_->outline(-1), maliput::common::assertion_error);
  EXPECT_THROW(dut_->outline(1), maliput::common::assertion_error);
}

TEST_F(RoadObjectTest, BoundingBox) {
  const auto& bb = dut_->bounding_box();
  EXPECT_DOUBLE_EQ(bb.position().x(), 1.);
  EXPECT_DOUBLE_EQ(bb.position().y(), 2.);
  EXPECT_DOUBLE_EQ(bb.position().z(), 3.);
  EXPECT_DOUBLE_EQ(bb.box_size().x(), 2.);
  EXPECT_DOUBLE_EQ(bb.box_size().y(), 1.);
  EXPECT_DOUBLE_EQ(bb.box_size().z(), 0.5);
}

TEST_F(RoadObjectTest, Properties) {
  const auto& props = dut_->properties();
  EXPECT_EQ(static_cast<int>(props.size()), 2);
  EXPECT_EQ(props.at("material"), "steel");
  EXPECT_EQ(props.at("source_id"), "obj_42");
}

TEST_F(RoadObjectTest, Orientation) {
  const auto& rot = dut_->orientation();
  EXPECT_DOUBLE_EQ(rot.rpy().roll_angle(), 0.);
  EXPECT_DOUBLE_EQ(rot.rpy().pitch_angle(), 0.);
  EXPECT_DOUBLE_EQ(rot.rpy().yaw_angle(), 1.57);
}

// Test with no optional fields.
GTEST_TEST(RoadObjectMinimalTest, NoOptionalFields) {
  const RoadObject::Id id("minimal_obj");
  const RoadObjectPosition position(InertialPosition(0., 0., 0.));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const maliput::math::BoundingBox bounding_box(math::Vector3(0., 0., 0.), math::Vector3(1., 1., 1.),
                                                math::RollPitchYaw(0., 0., 0.), 0.01);

  TestRoadObject dut(id, RoadObjectType::kUnknown, position, orientation, bounding_box, false, {} /* related_lanes */,
                     std::nullopt /* name */, std::nullopt /* subtype */, {} /* outlines */, {} /* properties */);

  EXPECT_EQ(dut.id().string(), "minimal_obj");
  EXPECT_FALSE(dut.name().has_value());
  EXPECT_EQ(dut.type(), RoadObjectType::kUnknown);
  EXPECT_FALSE(dut.subtype().has_value());
  EXPECT_FALSE(dut.is_dynamic());
  EXPECT_TRUE(dut.related_lanes().empty());
  EXPECT_EQ(dut.num_outlines(), 0);
  EXPECT_TRUE(dut.properties().empty());
}

// Test dynamic object.
GTEST_TEST(RoadObjectDynamicTest, IsDynamic) {
  const RoadObject::Id id("dynamic_obj");
  const RoadObjectPosition position(InertialPosition(0., 0., 0.));
  const Rotation orientation = Rotation::FromRpy(0., 0., 0.);
  const maliput::math::BoundingBox bounding_box(math::Vector3(0., 0., 0.), math::Vector3(1., 1., 1.),
                                                math::RollPitchYaw(0., 0., 0.), 0.01);

  TestRoadObject dut(id, RoadObjectType::kObstacle, position, orientation, bounding_box, true /* is_dynamic */,
                     {} /* related_lanes */, "Gate" /* name */, std::nullopt /* subtype */, {} /* outlines */,
                     {} /* properties */);

  EXPECT_TRUE(dut.is_dynamic());
  ASSERT_TRUE(dut.name().has_value());
  EXPECT_EQ(dut.name().value(), "Gate");
}

}  // namespace
}  // namespace objects
}  // namespace api
}  // namespace maliput
