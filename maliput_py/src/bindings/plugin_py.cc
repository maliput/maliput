#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "maliput/plugin/maliput_plugin.h"
#include "maliput/plugin/maliput_plugin_manager.h"

#include <map>
#include <string>

namespace maliput {
namespace bindings {

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
}

}  // namespace bindings
}  // namespace maliput
