#include "utilities/generate_obj.h"

#include <cmath>

#include <gtest/gtest.h>

#include "maliput/api/road_geometry.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace utility {
namespace test {

namespace common = maliput::common;

class MockGenerateObjTest : public ::testing::Test {
 protected:
  void SetUp() override {
    directory_.set_as_temp();
    directory_.append("MockGenerateObjTest");

    ASSERT_TRUE(common::Filesystem::create_directory(directory_));
  }

  void TearDown() override {
    // filesystem has no functionality for reading/walking a
    // directory, so we have to keep track of created files manually and
    // delete them by hand.
    for (const common::Path& path : paths_to_cleanup_) {
      EXPECT_TRUE(common::Filesystem::remove_file(path));
    }
    ASSERT_TRUE(common::Filesystem::remove_directory(directory_));
  }

  void ReadAsString(const common::Path& path, std::string* destination) {
    ASSERT_TRUE(common::Filesystem::read_as_string(path, *destination)) << "failed to read " << path.get_path();
  }

  common::Path directory_;
  std::vector<common::Path> paths_to_cleanup_;
};

TEST_F(MockGenerateObjTest, TwoLanesRoadGeometry) {
  const std::unique_ptr<const api::RoadGeometry> dut = api::test::CreateTwoLanesRoadGeometry();

  const std::string basename{"TwoLanesRoadGeometry"};
  std::string expected_obj_contents;
  ReadAsString(basename + ".obj", &expected_obj_contents);
  std::string expected_mtl_contents;
  ReadAsString(basename + ".mtl", &expected_mtl_contents);

  ObjFeatures features;
  features.min_grid_resolution = 5.0;
  GenerateObjFile(dut.get(), directory_.get_path(), basename, features);

  common::Path actual_obj_path(directory_);
  actual_obj_path.append(basename + ".obj");
  EXPECT_TRUE(actual_obj_path.is_file());
  paths_to_cleanup_.push_back(actual_obj_path);

  common::Path actual_mtl_path(directory_);
  actual_mtl_path.append(basename + ".mtl");
  EXPECT_TRUE(actual_mtl_path.is_file());
  paths_to_cleanup_.push_back(actual_mtl_path);

  // Quick regression test on the OBJ and MTL.
  std::string actual_obj_contents;
  ReadAsString(actual_obj_path, &actual_obj_contents);
  EXPECT_EQ(expected_obj_contents, actual_obj_contents);

  std::string actual_mtl_contents;
  ReadAsString(actual_mtl_path, &actual_mtl_contents);
  EXPECT_EQ(expected_mtl_contents, actual_mtl_contents);
}

}  // namespace test
}  // namespace utility
}  // namespace maliput
