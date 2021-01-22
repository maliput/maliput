// Copyright 2021 Toyota Research Institute

#include "maliput/utility/file_utils.h"

#include <stdlib.h>
#include <algorithm>
#include <string>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/common/filesystem.h"

namespace maliput {
namespace utilty {
namespace {

// Creates four files:
//
// /tmp
// ├── /file_utils
// │   ├── dumb_file_1.txt
// │   ├── dumb_file_2.txt
// │   └── /child_folder
// │       ├── dumb_file_1.so
// │       └── dumb_file_2.so
class GetAllFilePathsFromDirectoryTest : public ::testing::Test {
 public:
  void SetUp() override {
    base_path.set_as_temp();
    base_path.append("file_utils");
    ASSERT_TRUE(common::Filesystem::create_directory_recursive(base_path));
    std::string suffix{"txt"};
    CreateTwoFilesInBasePath(base_path, suffix);
    child_folder_path = {base_path.get_path() + "/child_folder"};
    common::Filesystem::create_directory(child_folder_path);
    suffix = "so";
    CreateTwoFilesInBasePath(child_folder_path, suffix);
  }

  void TearDown() override {
    for (const common::Path& path : expected_filepaths) {
      ASSERT_TRUE(common::Filesystem::remove_file(path));
    }
    ASSERT_TRUE(common::Filesystem::remove_directory(child_folder_path));
    ASSERT_TRUE(common::Filesystem::remove_directory(base_path));
  }

  common::Path base_path;
  common::Path child_folder_path;
  std::vector<common::Path> expected_filepaths;

 private:
  // Creates two dumb files in `base_path` using a particular `suffix` for the files.
  void CreateTwoFilesInBasePath(const common::Path& base_path, const std::string& suffix) {
    const common::Path file_1_path = base_path.get_path() + "/dumb_file_1." + suffix;
    common::Filesystem::create_file(file_1_path, "dumb_file_1 content");
    expected_filepaths.push_back(file_1_path);
    const common::Path file_2_path = base_path.get_path() + "/dumb_file_2." + suffix;
    common::Filesystem::create_file(file_2_path, "dumb_file_2 content");
    expected_filepaths.push_back(file_2_path);
  }
};

TEST_F(GetAllFilePathsFromDirectoryTest, GetSuffix) {
  const std::string expected_suffix_0_1{"txt"};
  const std::string expected_suffix_2_3{"so"};
  EXPECT_EQ(expected_suffix_0_1, maliput::utility::GetSuffixFromPath(expected_filepaths[0].get_path()));
  EXPECT_EQ(expected_suffix_0_1, maliput::utility::GetSuffixFromPath(expected_filepaths[1].get_path()));
  EXPECT_EQ(expected_suffix_2_3, maliput::utility::GetSuffixFromPath(expected_filepaths[2].get_path()));
  EXPECT_EQ(expected_suffix_2_3, maliput::utility::GetSuffixFromPath(expected_filepaths[3].get_path()));
}

TEST_F(GetAllFilePathsFromDirectoryTest, CheckReturnedFilePaths) {
  {  // Not adding a suffix.
    const auto filepaths = maliput::utility::GetAllFilePathsFromDirectory(base_path.get_path(), std::nullopt);
    EXPECT_EQ(4, static_cast<int>(filepaths.size()));
    for (const auto& expected_filepath : expected_filepaths) {
      const auto it = std::find(filepaths.begin(), filepaths.end(), expected_filepath.get_path());
      EXPECT_NE(it, filepaths.end());
    }
  }
  {  // Adding a suffix.
    const std::string suffix{"so"};
    const auto filepaths = maliput::utility::GetAllFilePathsFromDirectory(base_path.get_path(), suffix);
    EXPECT_EQ(2, static_cast<int>(filepaths.size()));
    for (const auto& filepath : filepaths) {
      const auto it =
          std::find_if(expected_filepaths.begin(), expected_filepaths.end(), [&filepath](const common::Path& path) {
            return filepath == path.get_path() && maliput::utility::GetSuffixFromPath(path.get_path()) == "so";
          });
      EXPECT_NE(it, expected_filepaths.end());
    }
  }
}

// Creates a environment variable and add two paths to it.
class GetAllPathsFromEnvironmentTest : public ::testing::Test {
 public:
  void SetUp() override {
    const std::string env_val{kPaths[0] + ":" + kPaths[1]};
    ASSERT_TRUE(setenv(kEnvName.c_str(), env_val.c_str(), 1 /*replace*/) == 0);
  }
  void TearDown() override { ASSERT_TRUE(unsetenv(kEnvName.c_str()) == 0); }

  const std::string kEnvName{"TEST_PATHS_FROM_ENV"};
  const std::string kPaths[2]{{"/tmp/test/path/one"}, {"/tmp/test/path/two"}};
};

// Uses maliput::utility::GetAllPathsFromEnvironment() to get the environment variable value.
TEST_F(GetAllPathsFromEnvironmentTest, GetPaths) {
  const auto dut_result = maliput::utility::GetAllPathsFromEnvironment(kEnvName);
  for (const auto& expected_path : kPaths) {
    auto it = std::find(dut_result.begin(), dut_result.end(), expected_path);
    EXPECT_NE(it, dut_result.end());
  }
}

}  // namespace
}  // namespace utilty
}  // namespace maliput
