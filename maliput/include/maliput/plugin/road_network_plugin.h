#include <string>

#include "maliput/api/road_network.h"

namespace maliput {
namespace plugin {

/// Interface class for creating a RoadNetwork plugin.
class RoadNetworkPlugin {
 public:
  /// Returns a maliput::api::RoadNetwork.
  /// @param parameters Dictionary containing the arguments needed for creating the RoadNetwork.
  virtual std::unique_ptr<const maliput::api::RoadNetwork> LoadRoadNetwork(
      const std::map<std::string, std::string> parameters) const = 0;
  virtual ~RoadNetworkPlugin() = default;
};

}  // namespace plugin
}  // namespace maliput
