#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "maliput/plugin/load_road_network.h"

#include <map>
#include <string>

namespace maliput {
namespace bindings {

namespace py = pybind11;

PYBIND11_MODULE(plugin, m) {
  py::class_<plugin::LoadRoadNetworkPlugin>(m, "LoadRoadNetworkPlugin")
      .def(py::init<std::string, std::map<std::string, std::string>>())
      .def("GetRoadNetwork", &plugin::LoadRoadNetworkPlugin::GetRoadNetwork);
}

}  // namespace bindings
}  // namespace maliput
