#include <string>

#include "maliput/api/road_network.h"

namespace maliput {
namespace plugin {

class RoadNetworkPlugin {
 public:
  virtual std::unique_ptr<const maliput::api::RoadNetwork> LoadRoadNetwork(
      const std::map<std::string, std::string> parameters) const = 0;
  virtual ~RoadNetworkPlugin() = default;
};

}  // namespace plugin
}  // namespace maliput
