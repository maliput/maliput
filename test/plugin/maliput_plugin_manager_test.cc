// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
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
// This test file is highly coupled to the maliput::dumb_plugin_x library creation.
// By using the aforementioned libraries, maliput::plugin::MaliputPluginManager class is tested.

#include "maliput/plugin/maliput_plugin_manager.h"

#include <algorithm>
#include <string>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/common/filesystem.h"

namespace maliput {
namespace plugin {
namespace {

// Environment variable containint the path where the plugins are loaded from is modified
// to point to a temporary location. There, shared libraries are installed so the
// the MaliputPluginManager's constructor is later on, able to load them.
class MaliputPluginManagerTest : public ::testing::Test {
 public:
  // Holds Id and custom method name from a MaliputPlugin.
  struct PluginFeatures {
    MaliputPlugin::Id id{"none"};
    std::string custom_method{};
  };

  // Creates a back of the current environment variable value to be restored in the tear down process.
  // Adds a temporary path where the test plugins are already install.
  void SetUp() override {
    back_up_env_ = common::Filesystem::get_env_path(kEnvName);
    ASSERT_TRUE(setenv(kEnvName.c_str(), kTestPluginsPath.c_str(), 1 /*replace*/) == 0);
  }

  // Restore the environment variable.
  void TearDown() override { ASSERT_TRUE(setenv(kEnvName.c_str(), back_up_env_.c_str(), 1 /*replace*/) == 0); }

  const std::string kEnvName{"MALIPUT_PLUGIN_PATH"};
  // The macro that this uses is provided as a compile definition in the
  // CMakeLists.txt file.
  // @{
  const std::string kTestPluginsPath{TEST_MALIPUT_PLUGIN_LIBDIR};
  const std::string kOtherTestPluginsPath{OTHER_TEST_MALIPUT_PLUGIN_LIBDIR};
  // }@

 private:
  std::string back_up_env_;
};

// Plugins are loaded and checked that their methods are working.
TEST_F(MaliputPluginManagerTest, ConstructorGetAndListPlugin) {
  MaliputPluginManager dut;
  const PluginFeatures kPlugin1{MaliputPlugin::Id("multiply_integers_test_plugin"), "MultiplyIntegers"};
  const PluginFeatures kPlugin2{MaliputPlugin::Id("sum_integers_test_plugin"), "SumIntegers"};
  const PluginFeatures kPlugin3{MaliputPlugin::Id("road_network_loader_test_plugin"), "GetMaliputPluginId"};

  const auto plugin_names = dut.ListPlugins();
  EXPECT_EQ(3, static_cast<int>(plugin_names.size()));
  EXPECT_NE(plugin_names.end(), std::find(plugin_names.begin(), plugin_names.end(), kPlugin1.id));
  EXPECT_NE(plugin_names.end(), std::find(plugin_names.begin(), plugin_names.end(), kPlugin2.id));

  // multiply_integers_test_plugin plugin.
  auto plugin = dut.GetPlugin(kPlugin1.id);
  EXPECT_EQ(kPlugin1.id.string(), plugin->GetId());
  int result{};
  ASSERT_NO_THROW(result = plugin->ExecuteSymbol<int>(kPlugin1.custom_method, 5, 10));
  EXPECT_EQ(50, result);

  // sum_integers_test_plugin plugin.
  plugin = dut.GetPlugin(kPlugin2.id);
  EXPECT_EQ(kPlugin2.id.string(), plugin->GetId());
  ASSERT_NO_THROW(result = plugin->ExecuteSymbol<int>(kPlugin2.custom_method, 5, 10));
  EXPECT_EQ(15, result);

  // road_network_loader_test_plugin plugin.
  char* str_result{};
  plugin = dut.GetPlugin(kPlugin3.id);
  EXPECT_EQ(kPlugin3.id.string(), plugin->GetId());
  ASSERT_NO_THROW(str_result = plugin->ExecuteSymbol<char*>(kPlugin3.custom_method));
  EXPECT_EQ(kPlugin3.id.string(), str_result);
}

// A plugin with the same ID is added in order to verify that a replacement is
// effectively done.
TEST_F(MaliputPluginManagerTest, RepeatedPlugin) {
  MaliputPluginManager dut;
  const PluginFeatures kPlugin3{MaliputPlugin::Id("multiply_integers_test_plugin"), "GetAString"};
  const std::string lib_path{kOtherTestPluginsPath + "libmaliput_lorem_ipsum_test_plugin.so"};
  dut.AddPlugin(lib_path);
  const auto plugin = dut.GetPlugin(kPlugin3.id);
  EXPECT_EQ(kPlugin3.id.string(), plugin->GetId());
  std::string result{};
  ASSERT_NO_THROW(result = plugin->ExecuteSymbol<char*>(kPlugin3.custom_method));
  EXPECT_EQ("LoremIpsum", result);
}

}  // namespace
}  // namespace plugin
}  // namespace maliput
