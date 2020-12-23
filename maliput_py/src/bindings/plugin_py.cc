#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "maliput/plugin/road_network_plugin_loader.h"

#include <map>
#include <string>

namespace maliput {
namespace bindings {

namespace py = pybind11;

PYBIND11_MODULE(plugin, m) {
  py::class_<plugin::RoadNetworkPluginLoader>(m, "RoadNetworkPluginLoader")
      .def(py::init<std::string, std::map<std::string, std::string>>())
      .def("GetRoadNetwork", &plugin::RoadNetworkPluginLoader::GetRoadNetwork);
}

}  // namespace bindings
}  // namespace maliput
