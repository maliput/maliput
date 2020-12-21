#pragma once

#include <map>
#include <memory>
#include <string>

#include "maliput/api/road_network.h"

namespace maliput {
namespace plugin {

class LoadRoadNetworkPlugin {
 public:
  LoadRoadNetworkPlugin(const std::string& lib_name, const std::map<std::string, std::string>& parameters);

  std::unique_ptr<const api::RoadNetwork> GetRoadNetwork();
  ~LoadRoadNetworkPlugin();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace plugin
}  // namespace maliput
