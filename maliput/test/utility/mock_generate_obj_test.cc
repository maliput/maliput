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
#include <memory>

#include <gtest/gtest.h>

#include "maliput/api/road_geometry.h"
#include "maliput/api/road_network.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/utility/generate_obj.h"

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

// OBJ and MTL files generated from the GeneratedObjFile method are compared with the following files
// located in the test path of maliput::utility' tests:
//  - TwoLanesRoadGeometry.mtl
//  - TwoLanesRoadGeometry.obj
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

// OBJ and MTL files generated from the GeneratedObjFile method are compared with the following files
// located in the test path of maliput::utility' tests:
//  - TwoLanesRoadGeometry.mtl
//  - TwoLanesRoadGeometry.obj
// Tests are run with an api::RoadNetwork instead of a api::RoadGeometry.
TEST_F(MockGenerateObjTest, TwoLanesRoadNetwork) {
  auto dut = std::make_unique<api::RoadNetwork>(
      api::test::CreateTwoLanesRoadGeometry(), api::test::CreateRoadRulebook(), api::test::CreateTrafficLightBook(),
      api::test::CreateIntersectionBook(), api::test::CreatePhaseRingBook(),
      api::test::CreateRightOfWayRuleStateProvider(), api::test::CreatePhaseProvider(), api::test::CreateRuleRegistry(),
      api::test::CreateDiscreteValueRuleStateProvider(), api::test::CreateRangeValueRuleStateProvider());

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
