// Copyright 2021 Toyota Research Institute
// @file
// This test file is highly coupled to the maliput::dumb_plugin_x library creation.
// By using the aforementioned libraries, maliput::plugin::MaliputPluginManager class is tested.

#include "maliput/plugin/maliput_plugin_manager.h"

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
    back_up_env = common::Filesystem::get_env_path(kEnvName);
    ASSERT_TRUE(setenv(kEnvName.c_str(), kTestPluginsPath.c_str(), 1 /*replace*/) == 0);
  }

  // Restore the environment variable.
  void TearDown() override { ASSERT_TRUE(setenv(kEnvName.c_str(), back_up_env.c_str(), 1 /*replace*/) == 0); }

  const std::string kEnvName{"MALIPUT_PLUGIN_PATH"};
  // The macro that this uses is provided as a compile definition in the
  // CMakeLists.txt file.
  // @{
  const std::string kTestPluginsPath{TEST_MALIPUT_PLUGIN_LIBDIR};
  const std::string kOtherTestPluginsPath{OTHER_TEST_MALIPUT_PLUGIN_LIBDIR};
  // }@

 private:
  std::string back_up_env;
};

// Plugins are loaded and checked that their methods are working.
TEST_F(MaliputPluginManagerTest, ConstructorAndGetPlugin) {
  MaliputPluginManager dut;
  const PluginFeatures kPlugin1{MaliputPlugin::Id("dumb_plugin_1"), "MultiplyIntegers"};
  const PluginFeatures kPlugin2{MaliputPlugin::Id("dumb_plugin_2"), "SumIntegers"};
  auto plugin = dut.GetPlugin(kPlugin1.id);
  EXPECT_EQ(kPlugin1.id.string(), plugin->GetId());
  EXPECT_NO_THROW(plugin->ExecuteSymbol<int>(kPlugin1.custom_method, 5, 10));

  plugin = dut.GetPlugin(kPlugin2.id);
  EXPECT_EQ(kPlugin2.id.string(), plugin->GetId());
  EXPECT_NO_THROW(plugin->ExecuteSymbol<int>(kPlugin2.custom_method, 5, 10));
}

// A plugin with the same ID is added in order to verify that a replacement is
// effectively done.
TEST_F(MaliputPluginManagerTest, RepeatedPlugin) {
  MaliputPluginManager dut;
  const PluginFeatures kPlugin3{MaliputPlugin::Id("dumb_plugin_1"), "GetAString"};

  const std::string lib_path{kOtherTestPluginsPath + "libmaliput_dumb_plugin_3.so"};
  dut.AddPlugin(lib_path);
  const auto plugin = dut.GetPlugin(kPlugin3.id);
  EXPECT_EQ(kPlugin3.id.string(), plugin->GetId());
  EXPECT_EQ("LoremIpsum", plugin->ExecuteSymbol<std::string>(kPlugin3.custom_method));
}

}  // namespace
}  // namespace plugin
}  // namespace maliput
