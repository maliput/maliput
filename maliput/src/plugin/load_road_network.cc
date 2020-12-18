#include "maliput/plugin/load_road_network.h"

#include "maliput/plugin/road_network_plugin.h"
#include "plugin_loader.h"

namespace maliput {
namespace plugin {

class LoadRoadNetworkPlugin::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl);
  Impl(const std::string& lib_name)
      : plugin_loader_(std::make_unique<plugin::PluginLoader<plugin::RoadNetworkPlugin>>(lib_name)) {
    road_network_plugin_ = plugin_loader_->GetInstance();
  }

  std::unique_ptr<const api::RoadNetwork> GetRoadNetwork() { return road_network_plugin_->LoadRoadNetwork(); }

  ~Impl() = default;

 private:
  std::unique_ptr<plugin::PluginLoader<plugin::RoadNetworkPlugin>> plugin_loader_;
  std::unique_ptr<plugin::RoadNetworkPlugin> road_network_plugin_;
};

LoadRoadNetworkPlugin::LoadRoadNetworkPlugin(const std::string& lib_name) : impl_(std::make_unique<Impl>(lib_name)) {}

LoadRoadNetworkPlugin::~LoadRoadNetworkPlugin() = default;

std::unique_ptr<const api::RoadNetwork> LoadRoadNetworkPlugin::GetRoadNetwork() { return impl_->GetRoadNetwork(); }

}  // namespace plugin
}  // namespace maliput
