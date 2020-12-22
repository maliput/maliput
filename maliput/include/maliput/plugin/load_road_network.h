#pragma once

#include <map>
#include <memory>
#include <string>

#include "maliput/api/road_network.h"

namespace maliput {
namespace plugin {

/// Loads a RoadNetworkPlugin.
class LoadRoadNetworkPlugin {
 public:
  /// Loads a RoadNetworkPlugin.
  /// @param lib_name Name of the dynamic library that contains a RoadNetwork implementation.
  /// @param parameters A dictionary containing the arguments needed for building a RoadNetwork.
  LoadRoadNetworkPlugin(const std::string& lib_name, const std::map<std::string, std::string>& parameters);

  /// Get a maliput::api::RoadNetwork.
  std::unique_ptr<const api::RoadNetwork> GetRoadNetwork();
  ~LoadRoadNetworkPlugin();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace plugin
}  // namespace maliput
