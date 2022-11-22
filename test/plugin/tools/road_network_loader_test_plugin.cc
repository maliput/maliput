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
// This file is a basic maliput RoadNetworkLoader type plugin implementation which aims to
// be a test tool in order to verify the correct behaviour of maliput plugin architecture.
// This file shall not be modified.

#include <memory>
#include <string>

#include "maliput/common/maliput_unused.h"
#include "maliput/plugin/road_network_loader.h"

namespace maliput {
namespace plugin {
namespace {

// Mock implementation of a maliput::plugin::RoadNetworkLoader.
class MyRoadNetworkLoader : public maliput::plugin::RoadNetworkLoader {
 public:
  std::unique_ptr<maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const override {
    maliput::common::unused(properties);
    // For the purpose of this test, we return a nullptr RoadNetwork.
    return nullptr;
  }

  std::map<std::string, std::string> GetDefaultParameters() const override {
    return {{"configuration_1", "value_1"}, {"configuration_2", "value_2"}};
  }
};

}  // namespace

REGISTER_ROAD_NETWORK_LOADER_PLUGIN("road_network_loader_test_plugin", MyRoadNetworkLoader);

}  // namespace plugin
}  // namespace maliput
