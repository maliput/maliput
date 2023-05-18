// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_object/loader/loader.h"

#include <fstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>
#include <maliput/common/assertion_error.h>
#include <maliput/math/bounding_box.h>
#include <maliput/math/bounding_region.h>
#include <maliput/math/matrix.h>
#include <maliput/math/roll_pitch_yaw.h>
#include <maliput/math/vector.h>

#include "maliput/common/filesystem.h"
#include "maliput_object/api/object.h"
#include "maliput_object/api/object_book.h"

namespace maliput {
namespace object {
namespace loader {
namespace test {
namespace {

using maliput::math::BoundingBox;
using maliput::math::BoundingRegion;
using maliput::object::api::Object;

// @{ Asserts that no file or no string with YAML content throws.
TEST(LoadTest, EmptyString) { ASSERT_THROW(Load(""), std::runtime_error); }

TEST(LoadFileTest, EmptyString) { ASSERT_THROW(LoadFile(""), std::runtime_error); }
// @}

// Test structure to verify schema errors.
struct SchemaParserTestCase {
  std::string constraint{};
  std::string yaml_under_test{};
};

// Just to improve gtest error report.
std::ostream& operator<<(std::ostream& os, const SchemaParserTestCase& test_case) {
  os << "{ constraint: " << test_case.constraint << ", yaml_under_test: " << test_case.yaml_under_test << "}";
  return os;
}

// Returns a list of test cases for the schema assertions.
std::vector<SchemaParserTestCase> GetYamlsWithSchemaErrors() {
  return {
      {"no maliput objects key", fmt::format(
                                     R"R(---
a_maliput_key: \"Hello world!\"
anoter_maliput_key:
  still_not_valid_key:
    - 1
    - 2
    - 3 
)R")},
      {"maliput_objects must be a mapping", fmt::format(
                                                R"R(---
maliput_objects:
  - 1
  - 2
)R")},
      {"objects must contain a bounding_region", fmt::format(
                                                     R"R(---
maliput_objects:
  an_object:
    not_a_bounding_region:
      position: [1, 2, 3]
      rotation: [0.4, 0.5, 0.6]
      type: box
      box_size: [0.7, 0.8, 0.9]
    properties:
      a_key: a_value
)R")},
      {"bounding_region must contain a position", fmt::format(
                                                      R"R(---
maliput_objects:
  an_object:
    bounding_region:
      not_a_position: [1, 2, 3]
      rotation: [0.4, 0.5, 0.6]
      type: box
      box_size: [0.7, 0.8, 0.9]
    properties:
      a_key: a_value
)R")},
      {"bounding_region must contain a rotation", fmt::format(
                                                      R"R(---
maliput_objects:
  an_object:
    bounding_region:
      not_a_position: [1, 2, 3]
      rotation: [0.4, 0.5, 0.6]
      type: box
      box_size: [0.7, 0.8, 0.9]
    properties:
      a_key: a_value
)R")},
      {"bounding_region must contain a type", fmt::format(
                                                  R"R(---
maliput_objects:
  an_object:
    bounding_region:
      position: [1, 2, 3]
      rotation: [0.4, 0.5, 0.6]
      not_a_type: box
      box_size: [0.7, 0.8, 0.9]
    properties:
      a_key: a_value
)R")},
      {"bounding_region must contain a type and must be a box", fmt::format(
                                                                    R"R(---
maliput_objects:
  an_object:
    bounding_region:
      position: [1, 2, 3]
      rotation: [0.4, 0.5, 0.6]
      type: not_box
      box_size: [0.7, 0.8, 0.9]
    properties:
      a_key: a_value
)R")},
      {"bounding_region must contain box_size", fmt::format(
                                                    R"R(---
maliput_objects:
  an_object:
    bounding_region:
      position: [1, 2, 3]
      rotation: [0.4, 0.5, 0.6]
      type: box
      not_box_size: [0.7, 0.8, 0.9]
    properties:
      a_key: a_value
)R")},
      {"position has no three elements", fmt::format(
                                             R"R(---
maliput_objects:
  an_object:
    bounding_region:
      position: [1., 2.]
      rotation: [0.4, 0.5, 0.6]
      type: box
      box_size: [0.7, 0.8, 0.9]
    properties:
      a_key: a_value
)R")},
      {"rotation has no three elements", fmt::format(
                                             R"R(---
maliput_objects:
  an_object:
    bounding_region:
      position: [1., 2., 3.]
      rotation: [0.4, 0.5, 0.6, 0.666]
      type: box
      box_size: [0.7, 0.8, 0.9]
    properties:
      a_key: a_value
)R")},
      {"box_size has no three elements", fmt::format(
                                             R"R(---
maliput_objects:
  an_object:
    bounding_region:
      position: [1., 2., 3.]
      rotation: [0.4, 0.5, 0.6, 0.666]
      type: box
      box_size: [0.7]
    properties:
      a_key: a_value
)R")},
  };
}

class SchemaParserCheckTest : public ::testing::TestWithParam<SchemaParserTestCase> {};

TEST_P(SchemaParserCheckTest, AssertsThatItThrowsWhenSchemaIsIncorrect) {
  ASSERT_THROW(Load(GetParam().yaml_under_test), maliput::common::assertion_error);
}

INSTANTIATE_TEST_CASE_P(SchemaParserCheckTestGroup, SchemaParserCheckTest,
                        ::testing::ValuesIn(GetYamlsWithSchemaErrors()));

struct ObjectTestFeatures {
  static constexpr const char* kObjectId{"object_id"};
  static constexpr double kX{1.0};
  static constexpr double kY{2.0};
  static constexpr double kZ{3.0};
  static constexpr double kRoll{.4};
  static constexpr double kPitch{.5};
  static constexpr double kYaw{.6};
  static constexpr double kLength{7.};
  static constexpr double kDepth{8.};
  static constexpr double kHeight{9.};
  static constexpr const char* kKey{"a_key"};
  static constexpr const char* kValue{"a value"};

  ObjectTestFeatures() = default;
  virtual ~ObjectTestFeatures() = default;

  static std::string GenerateYamlString() {
    return fmt::format(
        R"R(---
maliput_objects:
  {}:
    bounding_region:
      position: [{}, {}, {}]
      rotation: [{}, {}, {}]
      type: box
      box_size: [{}, {}, {}]
    properties:
      {}: "{}" 
)R",
        kObjectId, kX, kY, kZ, kRoll, kPitch, kYaw, kLength, kDepth, kHeight, kKey, kValue);
  }

  static void TestObjectBook(const api::ObjectBook<maliput::math::Vector3>* object_book) {
    ASSERT_NE(nullptr, object_book);
    ASSERT_EQ(1u, object_book->objects().size());

    const Object<maliput::math::Vector3>::Id id{kObjectId};
    Object<maliput::math::Vector3>* object = object_book->FindById(id);
    ASSERT_NE(nullptr, object);
    ASSERT_EQ(id, object->id());
    ASSERT_DOUBLE_EQ(kX, object->position().x());
    ASSERT_DOUBLE_EQ(kY, object->position().y());
    ASSERT_DOUBLE_EQ(kZ, object->position().z());
    ASSERT_EQ(1u, object->get_properties().size());
    ASSERT_EQ(kValue, object->get_property(kKey));

    const BoundingRegion<maliput::math::Vector3>& bounding_region = object->bounding_region();
    const BoundingBox* bounding_box = dynamic_cast<const BoundingBox*>(&bounding_region);
    ASSERT_NE(nullptr, bounding_box);
    ASSERT_DOUBLE_EQ(kX, bounding_box->position().x());
    ASSERT_DOUBLE_EQ(kY, bounding_box->position().y());
    ASSERT_DOUBLE_EQ(kZ, bounding_box->position().z());
    ASSERT_DOUBLE_EQ(kRoll, bounding_box->get_orientation().roll_angle());
    ASSERT_DOUBLE_EQ(kPitch, bounding_box->get_orientation().pitch_angle());
    ASSERT_DOUBLE_EQ(kYaw, bounding_box->get_orientation().yaw_angle());

    // Computes the position of the top-right corner.
    const maliput::math::Vector3 top_right_corner_position =
        maliput::math::RollPitchYaw(kRoll, kPitch, kYaw).ToMatrix().inverse() *
            maliput::math::Vector3(kLength / 2., kDepth / 2., kHeight / 2.) +
        maliput::math::Vector3(kX, kY, kZ);
    const maliput::math::Vector3 top_right_corner_position_ut = bounding_box->get_vertices().front();
    ASSERT_DOUBLE_EQ(top_right_corner_position.x(), top_right_corner_position_ut.x());
    ASSERT_DOUBLE_EQ(top_right_corner_position.y(), top_right_corner_position_ut.y());
    ASSERT_DOUBLE_EQ(top_right_corner_position.z(), top_right_corner_position_ut.z());
  }
};

TEST(LoadFromStringTest, OneValidObject) {
  std::unique_ptr<api::ObjectBook<maliput::math::Vector3>> object_book = Load(ObjectTestFeatures::GenerateYamlString());
  ObjectTestFeatures::TestObjectBook(object_book.get());
}

class LoadFromFileTest : public ::testing::Test {
 protected:
  void SetUp() override {
    directory_.set_as_temp();
    directory_.append("LoadObjectsFromYamlFileTest");
    ASSERT_TRUE(common::Filesystem::create_directory(directory_));

    filepath_ = directory_.get_path() + "/objects_test.yaml";
    GenerateYamlFileFromString(ObjectTestFeatures::GenerateYamlString(), filepath_);
  }

  void TearDown() override {
    if (!filepath_.empty()) {
      EXPECT_TRUE(common::Filesystem::remove_file(common::Path(filepath_)));
    }
    ASSERT_TRUE(common::Filesystem::remove_directory(directory_));
  }

  static void GenerateYamlFileFromString(const std::string& string_to_yaml, const std::string& filepath) {
    std::ofstream os(filepath);
    fmt::print(os, string_to_yaml);
  }

  maliput::common::Path directory_;
  std::string objects_string_;
  std::string filepath_;
};

TEST_F(LoadFromFileTest, EvaluateLoadFromFile) {
  std::unique_ptr<api::ObjectBook<maliput::math::Vector3>> object_book = LoadFile(filepath_);
  ObjectTestFeatures::TestObjectBook(object_book.get());
}

}  // namespace
}  // namespace test
}  // namespace loader
}  // namespace object
}  // namespace maliput
