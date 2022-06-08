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
