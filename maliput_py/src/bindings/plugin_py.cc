#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "maliput/common/logger.h"
#include "maliput/plugin/maliput_plugin.h"
#include "maliput/plugin/maliput_plugin_manager.h"
#include "maliput/plugin/road_network_loader.h"

#include <map>
#include <string>

namespace maliput {
namespace bindings {
namespace {

// Creates a maliput::api::RoadNetwork.
// @param plugin_id Plugin implementation id to be used.
// @param properties A dictionary containing configuration parameters for the road network builder.
// @returns A maliput::api::RoadNetwork.
//
// @throws maliput::common::assertion_error When `plugin_id` is not found.
// @throws maliput::common::assertion_error When the plugin isn't a RoadNetworkLoader plugin type.
// @throws maliput::common::assertion_error When the RoadNetwork can't be loaded.
std::unique_ptr<const maliput::api::RoadNetwork> CreateRoadNetworkFromPlugin(
    const std::string& plugin_id, const std::map<std::string, std::string>& properties) {
  // 'manager' is static for two main reasons:
  // 1 - The manager should keep loaded the correspondant plugin until the program is finished.
  // 2 - There is no need to reload the libraries every time this function is called.
  static plugin::MaliputPluginManager manager{};
  const plugin::MaliputPlugin* maliput_plugin = manager.GetPlugin(plugin::MaliputPlugin::Id(plugin_id));
  if (!maliput_plugin) {
    maliput::log()->error("{} plugin hasn't been found.", plugin_id);
    MALIPUT_THROW_MESSAGE(plugin_id + " plugin hasn't been found.");
  }
  if (maliput_plugin->GetType() != plugin::MaliputPluginType::kRoadNetworkLoader) {
    maliput::log()->error("{} plugin should be a RoadNetworkLoader plugin type", plugin_id);
    MALIPUT_THROW_MESSAGE(plugin_id + " plugin should be a RoadNetworkLoader plugin type.");
  }
  std::unique_ptr<maliput::plugin::RoadNetworkLoader> road_network_loader =
      maliput_plugin->ExecuteSymbol<std::unique_ptr<maliput::plugin::RoadNetworkLoader>>(
          maliput::plugin::RoadNetworkLoader::GetEntryPoint());
  return (*road_network_loader)(properties);
}

}  // namespace

namespace py = pybind11;

PYBIND11_MODULE(plugin, m) {
  py::class_<plugin::MaliputPlugin>(m, "MaliputPlugin")
      .def(py::init<std::string>())
      .def("GetId", &plugin::MaliputPlugin::GetId)
      .def("GetType", &plugin::MaliputPlugin::GetType);

  py::class_<plugin::MaliputPluginManager>(m, "MaliputPluginManager")
      .def(py::init<>())
      .def("GetPlugin", &plugin::MaliputPluginManager::GetPlugin, py::return_value_policy::reference_internal)
      .def("AddPlugin", &plugin::MaliputPluginManager::AddPlugin);

  m.def("create_road_network_from_plugin", &CreateRoadNetworkFromPlugin,
        "Creates a maliput::api::plugin::RoadNetwork using `plugin_id` implementation.", py::arg("plugin_id"),
        py::arg("properties"));
}

}  // namespace bindings
}  // namespace maliput
