#include "maliput-utilities/generate_urdf.h"

#include <cmath>

#include <gtest/gtest.h>

#include "maliput/common/filesystem.h"
#include "multilane/builder.h"
#include "multilane/loader.h"

namespace maliput {
namespace utility {

namespace multi = maliput::multilane;
namespace common = maliput::common;

class GenerateUrdfTest : public ::testing::Test {
 protected:
  const std::string kJunkBasename{"junk"};

  void SetUp() override {
    directory_.set_as_temp();
    directory_.append("GenerateUrdfTest");

    ASSERT_TRUE(common::Filesystem::create_directory(directory_));
  }

  void TearDown() override { ASSERT_TRUE(common::Filesystem::remove_directory(directory_)); }

  common::Path directory_;
};

TEST_F(GenerateUrdfTest, AtLeastRunIt) {
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const double kLaneWidth = 2.;
  const double kShoulder = 1.;
  const api::HBounds kElevationBounds{0., 5.};
  const double kScaleLength = 1.;
  const multi::ComputationPolicy kComputationPolicy = multi::ComputationPolicy::kPreferAccuracy;
  const multi::LaneLayout kLaneLayout{kShoulder, kShoulder, 1 /* num_lanes */, 0 /* ref_lane */, 0. /* ref_r0 */};
  auto b = multi::BuilderFactory().Make(kLaneWidth, kElevationBounds, kLinearTolerance, kAngularTolerance, kScaleLength,
                                        kComputationPolicy);

  const multi::EndpointZ kZeroZ{0., 0., 0., 0.};
  const multi::Endpoint start{{0., 0., 0.}, kZeroZ};

  b->Connect("0", kLaneLayout, multi::StartReference().at(start, multi::Direction::kForward), multi::LineOffset(10.),
             multi::EndReference().z_at(kZeroZ, multi::Direction::kForward));
  const std::unique_ptr<const api::RoadGeometry> dut = b->Build(api::RoadGeometryId{"dut"});

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
<robot name="dut">
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

}  // namespace utility
}  // namespace maliput
