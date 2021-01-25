// Copyright 2021 Toyota Research Institute
// @file
// This test file is highly coupled to the maliput::dumb_plugin_x library creation.
// By using the aforementioned library, maliput::plugin::MaliputPlugin class is tested.

#include "maliput/plugin/maliput_plugin.h"

#include <string>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace plugin {
namespace {

// MaliputPlugin's constructor receives an empty path.
GTEST_TEST(MaliputPlugin, EmptyPath) {
  const std::string kEmptyPath{""};
  EXPECT_THROW(MaliputPlugin{kEmptyPath}, maliput::common::assertion_error);
}

// MaliputPlugin's constructor receives a wrong path.
GTEST_TEST(MaliputPlugin, kWrongPath) {
  const std::string kWrongPath{"/wrong/library/path/lib.so"};
  EXPECT_THROW(MaliputPlugin{kWrongPath}, maliput::common::assertion_error);
}

// MaliputPlugin's methods are tested loading a correct library.
GTEST_TEST(MaliputPlugin, FunctionsCall) {
  const std::string kLibraryPath{"/tmp/maliput/test/plugins/libmaliput_multiply_integers_test_plugin.so"};
  const std::string kCustomSymbol{"MultiplyIntegers"};
  const std::string kWrongSymbol{"WrongSymbol"};
  const MaliputPlugin maliput_plugin{kLibraryPath};
  EXPECT_EQ("multiply_integers_test_plugin", maliput_plugin.GetId());
  EXPECT_EQ(MaliputPluginType::kRoadNetworkLoader, maliput_plugin.GetType());

  EXPECT_THROW(maliput_plugin.ExecuteSymbol<double>(kWrongSymbol), maliput::common::assertion_error);
  EXPECT_EQ(200, maliput_plugin.ExecuteSymbol<int>(kCustomSymbol, 10, 20));
}

}  // namespace
}  // namespace plugin
}  // namespace maliput
