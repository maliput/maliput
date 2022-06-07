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
// Copyright 2021 Toyota Research Institute

#include "maliput/utility/file_utils.h"

#include <stdlib.h>

#include <algorithm>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/common/filesystem.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace utilty {
namespace {

// Creates a file in `path` filled with `content`.
// @param path File path.
// @param content String content of the file.
// @throws maliput::common::assertion_error When `path` is empty.
void CreateFile(const std::string& path, const std::string& content) {
  MALIPUT_THROW_UNLESS(!path.empty());
  std::ofstream ofs{path};
  ofs << content;
  ofs.close();
}

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
    base_path_.set_as_temp();
    base_path_.append("file_utils");
    ASSERT_TRUE(common::Filesystem::create_directory_recursive(base_path_));
    CreateTwoFilesInBasePath(base_path_, kTxtSuffix);
    child_folder_path_ = {base_path_.get_path() + "/child_folder"};
    common::Filesystem::create_directory(child_folder_path_);
    CreateTwoFilesInBasePath(child_folder_path_, kSoSuffix);
  }

  void TearDown() override {
    for (const common::Path& path : expected_filepaths_) {
      ASSERT_TRUE(common::Filesystem::remove_file(path));
    }
    ASSERT_TRUE(common::Filesystem::remove_directory(child_folder_path_));
    ASSERT_TRUE(common::Filesystem::remove_directory(base_path_));
  }

  const std::string kTxtSuffix{"txt"};
  const std::string kSoSuffix{"so"};
  common::Path base_path_;
  common::Path child_folder_path_;
  std::vector<common::Path> expected_filepaths_;

 private:
  // Creates two dumb files in `base_path` using a particular `suffix` for the files.
  void CreateTwoFilesInBasePath(const common::Path& base_path, const std::string& suffix) {
    const common::Path file_1_path = base_path.get_path() + "/dumb_file_1." + suffix;
    CreateFile(file_1_path.get_path(), "dumb_file_1 content");
    expected_filepaths_.push_back(file_1_path);
    const common::Path file_2_path = base_path.get_path() + "/dumb_file_2." + suffix;
    CreateFile(file_2_path.get_path(), "dumb_file_2 content");
    expected_filepaths_.push_back(file_2_path);
  }
};

TEST_F(GetAllFilePathsFromDirectoryTest, GetSuffix) {
  EXPECT_EQ(kTxtSuffix, maliput::utility::GetSuffixFromPath(expected_filepaths_[0].get_path()));
  EXPECT_EQ(kTxtSuffix, maliput::utility::GetSuffixFromPath(expected_filepaths_[1].get_path()));
  EXPECT_EQ(kSoSuffix, maliput::utility::GetSuffixFromPath(expected_filepaths_[2].get_path()));
  EXPECT_EQ(kSoSuffix, maliput::utility::GetSuffixFromPath(expected_filepaths_[3].get_path()));
}

TEST_F(GetAllFilePathsFromDirectoryTest, NoSuffixPassed) {
  const auto filepaths = maliput::utility::GetAllFilePathsFromDirectory(base_path_.get_path(), std::nullopt);
  EXPECT_EQ(4, static_cast<int>(filepaths.size()));
  for (const auto& expected_filepath : expected_filepaths_) {
    const auto it = std::find(filepaths.begin(), filepaths.end(), expected_filepath.get_path());
    EXPECT_NE(it, filepaths.end());
  }
}

TEST_F(GetAllFilePathsFromDirectoryTest, PassingASuffix) {
  const auto filepaths = maliput::utility::GetAllFilePathsFromDirectory(base_path_.get_path(), kSoSuffix);
  EXPECT_EQ(2, static_cast<int>(filepaths.size()));
  for (const auto& filepath : filepaths) {
    const auto it = std::find_if(expected_filepaths_.begin(), expected_filepaths_.end(),
                                 [&filepath, suffix = this->kSoSuffix](const common::Path& path) {
                                   return filepath == path.get_path() &&
                                          maliput::utility::GetSuffixFromPath(path.get_path()) == suffix;
                                 });
    EXPECT_NE(it, expected_filepaths_.end());
  }
}

// Adds:
//  - an environment variable and add two paths to it.
//  - an empty environment variable.
class GetAllPathsFromEnvironmentTest : public ::testing::Test {
 public:
  void SetUp() override {
    const std::string env_val{kPaths[0] + ":" + kPaths[1]};
    ASSERT_TRUE(setenv(kEnvName.c_str(), env_val.c_str(), 1 /*replace*/) == 0);
    ASSERT_TRUE(setenv(kEmptyEnvName.c_str(), "", 1 /*replace*/) == 0);
  }
  void TearDown() override {
    ASSERT_TRUE(unsetenv(kEnvName.c_str()) == 0);
    ASSERT_TRUE(unsetenv(kEmptyEnvName.c_str()) == 0);
  }

  const std::string kEnvName{"TEST_PATHS_FROM_ENV"};
  const std::string kEmptyEnvName{"EMPTY_TEST_PATHS_FROM_ENV"};
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

TEST_F(GetAllPathsFromEnvironmentTest, GetPathsFromEmptyEnvVar) {
  const auto dut_result = maliput::utility::GetAllPathsFromEnvironment(kEmptyEnvName);
  EXPECT_EQ(0, static_cast<int>(dut_result.size()));
}

}  // namespace
}  // namespace utilty
}  // namespace maliput
