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
#include <cmath>

#include <gtest/gtest.h>

#include "maliput/api/road_geometry.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/utility/generate_urdf.h"

namespace maliput {
namespace utility {
namespace test {

namespace common = maliput::common;

class MockGenerateUrdfTest : public ::testing::Test {
 protected:
  const std::string kJunkBasename{"junk"};

  void SetUp() override {
    directory_.set_as_temp();
    directory_.append("GenerateUrdfTest");

    ASSERT_TRUE(common::Filesystem::create_directory(directory_));
  }

  // clang-format off
  void TearDown() override {
    ASSERT_TRUE(common::Filesystem::remove_directory(directory_));
  }
  // clang-format on

  common::Path directory_;
};

TEST_F(MockGenerateUrdfTest, AtLeastRunIt) {
  const std::unique_ptr<const api::RoadGeometry> dut = api::test::CreateTwoLanesRoadGeometry();

  GenerateUrdfFile(dut.get(), directory_.get_path(), kJunkBasename, ObjFeatures());
  // We expect to get three files out of this.

  common::Path expected_urdf(directory_);
  expected_urdf.append(kJunkBasename + ".urdf");
  EXPECT_TRUE(expected_urdf.is_file());

  common::Path expected_obj(directory_);
  expected_obj.append(kJunkBasename + ".obj");
  EXPECT_TRUE(expected_obj.is_file());

  common::Path expected_mtl(directory_);
  expected_mtl.append(kJunkBasename + ".mtl");
  EXPECT_TRUE(expected_mtl.is_file());

  // Quick regression test on the URDF, which is mostly static content.
  std::string actual_urdf_contents;

  ASSERT_TRUE(common::Filesystem::read_as_string(expected_urdf, actual_urdf_contents));
  EXPECT_EQ(R"R(<?xml version="1.0" ?>
<robot name="road_geometry">
  <link name="world"/>

  <joint name="world_to_road_joint" type="continuous">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="surface"/>
  </joint>

  <link name="surface">
    <visual name="v1">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="junk.obj" scale="1.0 1.0 1.0"/>
      </geometry>
    </visual>
  </link>
</robot>
)R",
            actual_urdf_contents);

  // filesystem has no functionality for reading/walking a
  // directory, so we have to delete all our files individually here where
  // we know the names.
  EXPECT_TRUE(common::Filesystem::remove_file(expected_urdf));
  EXPECT_TRUE(common::Filesystem::remove_file(expected_obj));
  EXPECT_TRUE(common::Filesystem::remove_file(expected_mtl));
}

}  // namespace test
}  // namespace utility
}  // namespace maliput
