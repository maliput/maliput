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
#pragma once

#include <map>
#include <memory>
#include <string>

#include "maliput/api/road_network.h"
#include "maliput/plugin/road_network_loader.h"

namespace maliput {
namespace plugin {

/// Creates a maliput::api::RoadNetwork via RoadNetworkLoader plugin.
/// @param road_network_loader_id RoadNetworkLoader plugin id to be used.
/// @param properties A dictionary containing configuration parameters for the road network builder.
/// @returns A maliput::api::RoadNetwork.
///
/// @throws maliput::common::assertion_error When `road_network_loader_id` is not found.
/// @throws maliput::common::assertion_error When the plugin isn't a RoadNetworkLoader plugin type.
/// @throws maliput::common::assertion_error When the `maliput::api::RoadNetwork` can't be loaded.
std::unique_ptr<maliput::api::RoadNetwork> CreateRoadNetwork(const std::string& road_network_loader_id,
                                                             const std::map<std::string, std::string>& properties);

/// Creates a maliput::plugin::RoadNetworkLoader using the specified plugin id.
/// @param road_network_loader_id RoadNetworkLoader plugin id to be used.
/// @returns A maliput::plugin::RoadNetworkLoader.
///
/// @throws maliput::common::assertion_error When `road_network_loader_id` is not found.
/// @throws maliput::common::assertion_error When the plugin isn't a RoadNetworkLoader plugin type.
std::unique_ptr<maliput::plugin::RoadNetworkLoader> MakeRoadNetworkLoader(const std::string& road_network_loader_id);

}  // namespace plugin
}  // namespace maliput
