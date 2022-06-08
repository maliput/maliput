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
#include <string>

#include "maliput/api/road_network.h"
#include "maliput/plugin/maliput_plugin_type.h"

/// @def REGISTER_ROAD_NETWORK_LOADER_PLUGIN(PluginName, RoadNetworkLoaderClass)
/// Macro for automating the creation of the correspondant functions for the correct
/// implementation of a RoadNetworkLoader plugin.
///
/// @param PluginName Is the name of the plugin and must be unique among all the plugins.
/// @param RoadNetworkLoaderClass Is the implementation of the maliput::plugin::RoadNetworkLoader.
#define REGISTER_ROAD_NETWORK_LOADER_PLUGIN(PluginName, RoadNetworkLoaderClass) \
  extern "C" char* GetMaliputPluginId() { return (char*)PluginName; }           \
  extern "C" maliput::plugin::MaliputPluginType GetMaliputPluginType() {        \
    return maliput::plugin::MaliputPluginType::kRoadNetworkLoader;              \
  }                                                                             \
  extern "C" maliput::plugin::RoadNetworkLoaderPtr MakeRoadNetworkLoader() { return new RoadNetworkLoaderClass(); }

namespace maliput {
namespace plugin {

/// Additional name for the `MakeRoadNetworkLoader` method's return type.
typedef void* RoadNetworkLoaderPtr;

/// Interface class for creating a RoadNetwork loader functor.
class RoadNetworkLoader {
 public:
  /// @returns The entry point method name for getting an instance of the class.
  static std::string GetEntryPoint() { return "MakeRoadNetworkLoader"; }

  /// Returns a maliput::api::RoadNetwork.
  /// @param properties Dictionary containing the arguments needed for creating the RoadNetwork.
  virtual std::unique_ptr<maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const = 0;
  virtual ~RoadNetworkLoader() = default;
};

}  // namespace plugin
}  // namespace maliput
