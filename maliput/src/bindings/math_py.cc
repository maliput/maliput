#include "pybind11/pybind11.h"

#include "maliput/math/vector.h"

namespace maliput {
namespace bindings {

namespace py = pybind11;

PYBIND11_MODULE(math, m) {
  py::class_<math::Vector3>(m, "Vector3")
      .def(py::init<double, double, double>())
      .def("x", py::overload_cast<>(&math::Vector3::x))
      .def("y", py::overload_cast<>(&math::Vector3::y))
      .def("z", py::overload_cast<>(&math::Vector3::z));
}

}  // namespace bindings
}  // namespace maliput
