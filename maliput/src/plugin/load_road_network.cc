#include "maliput/plugin/load_road_network.h"

#include "maliput/plugin/road_network_plugin.h"
#include "plugin_loader.h"

namespace maliput {
namespace plugin {

class LoadRoadNetworkPlugin::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl);
  Impl(const std::string& lib_name, const std::map<std::string, std::string>& parameters)
      : plugin_loader_(std::make_unique<plugin::PluginLoader<plugin::RoadNetworkPlugin>>(lib_name, factory_method)),
        parameters_(parameters) {
    road_network_plugin_ = plugin_loader_->GetInstance();
  }

  std::unique_ptr<const api::RoadNetwork> GetRoadNetwork() {
    return road_network_plugin_->LoadRoadNetwork(parameters_);
  }

  ~Impl() = default;

 private:
  static constexpr char const* factory_method{"LoadMaliputRoadNetwork"};
  const std::unique_ptr<plugin::PluginLoader<plugin::RoadNetworkPlugin>> plugin_loader_;
  const std::map<std::string, std::string> parameters_;
  std::unique_ptr<plugin::RoadNetworkPlugin> road_network_plugin_;
};

LoadRoadNetworkPlugin::LoadRoadNetworkPlugin(const std::string& lib_name,
                                             const std::map<std::string, std::string>& parameters)
    : impl_(std::make_unique<Impl>(lib_name, parameters)) {}

LoadRoadNetworkPlugin::~LoadRoadNetworkPlugin() = default;

std::unique_ptr<const api::RoadNetwork> LoadRoadNetworkPlugin::GetRoadNetwork() { return impl_->GetRoadNetwork(); }

}  // namespace plugin
}  // namespace maliput
