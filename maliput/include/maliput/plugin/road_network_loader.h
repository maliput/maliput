// Copyright 2021 Toyota Research Institute
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
  virtual std::unique_ptr<const maliput::api::RoadNetwork> operator()(
      const std::map<std::string, std::string>& properties) const = 0;
  virtual ~RoadNetworkLoader() = default;
};

}  // namespace plugin
}  // namespace maliput
