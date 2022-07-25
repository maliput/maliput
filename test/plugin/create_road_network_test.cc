// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
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

// @file
// This test file is highly coupled to the dumb_road_network_plugin library creation.

#include "maliput/plugin/create_road_network.h"

#include <algorithm>
#include <string>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/common/filesystem.h"

namespace maliput {
namespace plugin {
namespace {

class CreateRoadNetworkTest : public ::testing::Test {
 public:
  // Creates a back up of the current environment variable value to be restored in the tear down process.
  // Adds a temporary path where the test plugins are already install.
  void SetUp() override {
    back_up_env_ = common::Filesystem::get_env_path(kEnvName);
    ASSERT_TRUE(setenv(kEnvName.c_str(), kTestPluginsPath.c_str(), 1 /*replace*/) == 0);
  }
  // Restore the environment variable.
  void TearDown() override { ASSERT_TRUE(setenv(kEnvName.c_str(), back_up_env_.c_str(), 1 /*replace*/) == 0); }

  const std::string kEnvName{"MALIPUT_PLUGIN_PATH"};
  const std::string kTestMaliputPlugin{"road_network_loader_test_plugin"};
  // The path where the test plugins are installed, provided via compile definition.
  const std::string kTestPluginsPath{TEST_MALIPUT_PLUGIN_LIBDIR};

 private:
  std::string back_up_env_;
};

TEST_F(CreateRoadNetworkTest, Throws) {
  // No matching plugin id.
  EXPECT_THROW(CreateRoadNetwork("wrong_id", {}), maliput::common::assertion_error);
}

TEST_F(CreateRoadNetworkTest, CreateRoadNetwork) { ASSERT_NO_THROW(CreateRoadNetwork(kTestMaliputPlugin, {})); }

}  // namespace
}  // namespace plugin
}  // namespace maliput
