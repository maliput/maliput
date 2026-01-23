// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include <dlfcn.h>

#include <algorithm>
#include <cstring>
#include <string>

#include <gtest/gtest.h>
#include <link.h>

#include "maliput/common/error.h"
#include "maliput/common/filesystem.h"

namespace maliput {
namespace plugin {
namespace {

// Structure to hold search parameters and results for dl_iterate_phdr callback.
// Mirrors the internal implementation in maliput_plugin_manager.cc for testing purposes.
struct TestPluginSearchContext {
  std::string search_pattern;
  std::string found_path;
  bool found{false};
};

// Callback for dl_iterate_phdr() to find loaded libraries matching a pattern.
// Mirrors the internal implementation for testing purposes.
int TestFindLoadedPluginCallback(struct dl_phdr_info* info, size_t /* size */, void* data) {
  auto* context = static_cast<TestPluginSearchContext*>(data);
  if (info->dlpi_name != nullptr && std::strlen(info->dlpi_name) > 0) {
    const std::string lib_path(info->dlpi_name);
    if (lib_path.find(context->search_pattern) != std::string::npos) {
      context->found_path = lib_path;
      context->found = true;
      return 1;
    }
  }
  return 0;
}

// Helper function to find a loaded library by pattern.
// Mirrors the internal implementation for testing purposes.
std::string TestFindLoadedLibraryByPattern(const std::string& pattern) {
  TestPluginSearchContext context;
  context.search_pattern = pattern;
  dl_iterate_phdr(TestFindLoadedPluginCallback, &context);
  return context.found ? context.found_path : "";
}

// Environment variable containint the path where the plugins are loaded from is modified
// to point to a temporary location. There, shared libraries are installed so the
// the MaliputPluginManager's constructor is later on, able to load them.
class MaliputPluginManagerTest : public ::testing::Test {
 public:
  // Holds Id and custom method name from a MaliputPlugin.
  struct PluginFeatures {
    MaliputPlugin::Id id{"none"};
    std::string custom_method{};
    MaliputPluginType type{MaliputPluginType::kRoadNetworkLoader};
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

  auto it = plugin_names.find(kPlugin1.id);
  ASSERT_NE(plugin_names.end(), it);
  EXPECT_EQ(kPlugin1.type, it->second);
  it = plugin_names.find(kPlugin2.id);
  ASSERT_NE(plugin_names.end(), it);
  EXPECT_EQ(kPlugin2.type, it->second);
  it = plugin_names.find(kPlugin3.id);
  ASSERT_NE(plugin_names.end(), it);
  EXPECT_EQ(kPlugin3.type, it->second);

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

// Tests for dl_iterate_phdr-based library discovery functionality.
class FindLoadedLibraryTest : public ::testing::Test {
 protected:
  const std::string kTestPluginsPath{TEST_MALIPUT_PLUGIN_LIBDIR};
};

// Verifies that dl_iterate_phdr can find a library that has been loaded with dlopen.
TEST_F(FindLoadedLibraryTest, FindsLoadedLibraryByPattern) {
  const std::string kLibraryPath{kTestPluginsPath + "libmaliput_multiply_integers_test_plugin.so"};

  // Load the library first.
  void* handle = dlopen(kLibraryPath.c_str(), RTLD_LAZY | RTLD_LOCAL);
  ASSERT_NE(nullptr, handle) << "Failed to load library: " << dlerror();

  // Search for it using pattern matching.
  const std::string found_path = TestFindLoadedLibraryByPattern("multiply_integers_test_plugin");
  EXPECT_FALSE(found_path.empty());
  EXPECT_NE(std::string::npos, found_path.find("multiply_integers_test_plugin"));

  dlclose(handle);
}

// Verifies that pattern matching works with partial library names (handles name mangling).
TEST_F(FindLoadedLibraryTest, FindsLibraryWithPartialPattern) {
  const std::string kLibraryPath{kTestPluginsPath + "libmaliput_sum_integers_test_plugin.so"};

  void* handle = dlopen(kLibraryPath.c_str(), RTLD_LAZY | RTLD_LOCAL);
  ASSERT_NE(nullptr, handle) << "Failed to load library: " << dlerror();

  // Search using just the core name (without lib prefix or .so suffix).
  const std::string found_path = TestFindLoadedLibraryByPattern("sum_integers_test_plugin");
  EXPECT_FALSE(found_path.empty());
  EXPECT_NE(std::string::npos, found_path.find("sum_integers_test_plugin"));

  dlclose(handle);
}

// Verifies that searching for a non-existent pattern returns empty string.
TEST_F(FindLoadedLibraryTest, ReturnsEmptyForNonExistentPattern) {
  const std::string found_path = TestFindLoadedLibraryByPattern("non_existent_library_pattern_xyz");
  EXPECT_TRUE(found_path.empty());
}

// Verifies that after finding a library by pattern, we can get a handle using RTLD_NOLOAD.
TEST_F(FindLoadedLibraryTest, CanGetHandleFromFoundLibrary) {
  const std::string kLibraryPath{kTestPluginsPath + "libmaliput_multiply_integers_test_plugin.so"};

  // Load the library.
  void* preload_handle = dlopen(kLibraryPath.c_str(), RTLD_LAZY | RTLD_LOCAL);
  ASSERT_NE(nullptr, preload_handle) << "Failed to load library: " << dlerror();

  // Find it by pattern.
  const std::string found_path = TestFindLoadedLibraryByPattern("multiply_integers_test_plugin");
  ASSERT_FALSE(found_path.empty());

  // Get a handle using RTLD_NOLOAD with the found path.
  void* noload_handle = dlopen(found_path.c_str(), RTLD_LAZY | RTLD_LOCAL | RTLD_NOLOAD);
  EXPECT_NE(nullptr, noload_handle) << "Failed to get handle with RTLD_NOLOAD: " << dlerror();

  if (noload_handle != nullptr) {
    dlclose(noload_handle);
  }
  dlclose(preload_handle);
}

// Verifies that multiple loaded libraries can be distinguished by their patterns.
TEST_F(FindLoadedLibraryTest, DistinguishesMultipleLoadedLibraries) {
  const std::string kLibraryPath1{kTestPluginsPath + "libmaliput_multiply_integers_test_plugin.so"};
  const std::string kLibraryPath2{kTestPluginsPath + "libmaliput_sum_integers_test_plugin.so"};

  void* handle1 = dlopen(kLibraryPath1.c_str(), RTLD_LAZY | RTLD_LOCAL);
  void* handle2 = dlopen(kLibraryPath2.c_str(), RTLD_LAZY | RTLD_LOCAL);
  ASSERT_NE(nullptr, handle1) << "Failed to load library 1: " << dlerror();
  ASSERT_NE(nullptr, handle2) << "Failed to load library 2: " << dlerror();

  // Find each library by its specific pattern.
  const std::string found_path1 = TestFindLoadedLibraryByPattern("multiply_integers");
  const std::string found_path2 = TestFindLoadedLibraryByPattern("sum_integers");

  EXPECT_FALSE(found_path1.empty());
  EXPECT_FALSE(found_path2.empty());
  EXPECT_NE(found_path1, found_path2);
  EXPECT_NE(std::string::npos, found_path1.find("multiply"));
  EXPECT_NE(std::string::npos, found_path2.find("sum"));

  dlclose(handle1);
  dlclose(handle2);
}

}  // namespace
}  // namespace plugin
}  // namespace maliput
