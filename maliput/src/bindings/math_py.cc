#include "pybind11/pybind11.h"

#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace bindings {

namespace py = pybind11;

PYBIND11_MODULE(math, m) {
  py::class_<math::Vector3>(m, "Vector3")
      .def(py::init<double, double, double>())
      .def("__getitem__", py::overload_cast<std::size_t>(&math::Vector3::operator[]), py::is_operator())
      .def("x", py::overload_cast<>(&math::Vector3::x))
      .def("y", py::overload_cast<>(&math::Vector3::y))
      .def("z", py::overload_cast<>(&math::Vector3::z));
  py::class_<math::RollPitchYaw>(m, "RollPitchYaw")
      .def(py::init<double, double, double>())
      .def("roll_angle", py::overload_cast<>(&math::RollPitchYaw::roll_angle))
      .def("pitch_angle", py::overload_cast<>(&math::RollPitchYaw::pitch_angle))
      .def("yaw_angle", py::overload_cast<>(&math::RollPitchYaw::yaw_angle));
}

}  // namespace bindings
}  // namespace maliput
