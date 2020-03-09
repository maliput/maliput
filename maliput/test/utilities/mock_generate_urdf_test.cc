#include "utilities/generate_urdf.h"

#include <cmath>

#include <gtest/gtest.h>

#include "maliput/api/road_geometry.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/mock.h"

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
