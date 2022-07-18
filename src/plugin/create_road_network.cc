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
#include "maliput/plugin/create_road_network.h"

#include "maliput/common/logger.h"
#include "maliput/plugin/maliput_plugin.h"
#include "maliput/plugin/maliput_plugin_manager.h"
#include "maliput/plugin/road_network_loader.h"

namespace maliput {
namespace plugin {

std::unique_ptr<maliput::api::RoadNetwork> CreateRoadNetwork(const std::string& road_network_loader_id,
                                                             const std::map<std::string, std::string>& properties) {
  // 'manager' is static for two main reasons:
  // 1 - The manager should keep loaded the correspondent plugin until the program is finished.
  // 2 - There is no need to reload the libraries every time this function is called.
  static plugin::MaliputPluginManager manager{};
  const plugin::MaliputPlugin* maliput_plugin = manager.GetPlugin(plugin::MaliputPlugin::Id(road_network_loader_id));
  if (!maliput_plugin) {
    maliput::log()->error("{} plugin can't be obtained.", road_network_loader_id);
    MALIPUT_THROW_MESSAGE(road_network_loader_id + " plugin can't be obtained.");
  }
  if (maliput_plugin->GetType() != plugin::MaliputPluginType::kRoadNetworkLoader) {
    maliput::log()->error("{} plugin should be a RoadNetworkLoader plugin type", road_network_loader_id);
    MALIPUT_THROW_MESSAGE(road_network_loader_id + " plugin should be a RoadNetworkLoader plugin type.");
  }
  maliput::plugin::RoadNetworkLoaderPtr rn_loader_ptr =
      maliput_plugin->ExecuteSymbol<maliput::plugin::RoadNetworkLoaderPtr>(
          maliput::plugin::RoadNetworkLoader::GetEntryPoint());
  // Use smart pointers to gracefully manage heap allocation.
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> road_network_loader{
      reinterpret_cast<maliput::plugin::RoadNetworkLoader*>(rn_loader_ptr)};
  // Generates the maliput::api::RoadNetwork.
  return (*road_network_loader)(properties);
}

}  // namespace plugin
}  // namespace maliput
