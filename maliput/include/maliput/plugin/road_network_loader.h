// Copyright 2020 Toyota Research Institute
#include <string>

#include "maliput/api/road_network.h"
#include "maliput/plugin/maliput_plugin_type.h"

/// Macro for automating the creation of the correspondant functions for the correct
/// implementation of a RoadNetworkLoader plugin.
///
/// `PluginName` is the name of the plugin and must be unique among all the plugins.
/// `RoadNetworkLoaderClass` is the implementation of the maliput::plugin::RoadNetworkLoader.
#define REGISTER_ROAD_NETWORK_LOADER_PLUGIN(PluginName, RoadNetworkLoaderClass)            \
  extern "C" std::string GetMaliputPluginId() { return std::string(PluginName); }          \
  extern "C" maliput::plugin::MaliputPluginType GetMaliputPluginType() {                   \
    return maliput::plugin::MaliputPluginType::kRoadNetworkLoader;                         \
  }                                                                                        \
  extern "C" std::unique_ptr<maliput::plugin::RoadNetworkLoader> MakeRoadNetworkLoader() { \
    return std::make_unique<RoadNetworkLoaderClass>();                                     \
  }

namespace maliput {
namespace plugin {

/// Interface class for creating a RoadNetwork loader functor.
class RoadNetworkLoader {
 public:
  /// Returns a maliput::api::RoadNetwork.
  /// @param properties Dictionary containing the arguments needed for creating the RoadNetwork.
  virtual std::unique_ptr<const maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const = 0;
  virtual ~RoadNetworkLoader() = default;
};

}  // namespace plugin
}  // namespace maliput
