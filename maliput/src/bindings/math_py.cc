#include "pybind11/pybind11.h"

#include "maliput/math/quaternion.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace bindings {

namespace py = pybind11;

PYBIND11_MODULE(math, m) {
  py::class_<math::Vector3>(m, "Vector3")
      .def(py::init<double, double, double>())
      .def("__getitem__", py::overload_cast<std::size_t>(&math::Vector3::operator[]), py::is_operator())
      .def("__eq__", [](const math::Vector3& a, const math::Vector3& b) { return a == b; })
      .def("__ne__", [](const math::Vector3& a, const math::Vector3& b) { return a != b; })
      .def("size", &math::Vector3::size)
      .def("x", py::overload_cast<>(&math::Vector3::x))
      .def("y", py::overload_cast<>(&math::Vector3::y))
      .def("z", py::overload_cast<>(&math::Vector3::z));

  py::class_<math::Vector4>(m, "Vector4")
      .def(py::init<double, double, double, double>())
      .def("__getitem__", py::overload_cast<std::size_t>(&math::Vector4::operator[]), py::is_operator())
      .def("__eq__", [](const math::Vector4& a, const math::Vector4& b) { return a == b; })
      .def("__ne__", [](const math::Vector4& a, const math::Vector4& b) { return a != b; })
      .def("size", &math::Vector4::size)
      .def("x", py::overload_cast<>(&math::Vector4::x))
      .def("y", py::overload_cast<>(&math::Vector4::y))
      .def("z", py::overload_cast<>(&math::Vector4::z))
      .def("w", py::overload_cast<>(&math::Vector4::w));

  py::class_<math::RollPitchYaw>(m, "RollPitchYaw")
      .def(py::init<double, double, double>())
      .def("ToQuaternion", &math::RollPitchYaw::ToQuaternion)
      .def("roll_angle", py::overload_cast<>(&math::RollPitchYaw::roll_angle))
      .def("pitch_angle", py::overload_cast<>(&math::RollPitchYaw::pitch_angle))
      .def("yaw_angle", py::overload_cast<>(&math::RollPitchYaw::yaw_angle));

  py::class_<math::Quaternion>(m, "Quaternion")
      .def(py::init<double, double, double, double>())
      .def("coeffs", &math::Quaternion::coeffs)
      .def("w", py::overload_cast<>(&math::Quaternion::w))
      .def("x", py::overload_cast<>(&math::Quaternion::x))
      .def("y", py::overload_cast<>(&math::Quaternion::y))
      .def("z", py::overload_cast<>(&math::Quaternion::z));
}

}  // namespace bindings
}  // namespace maliput
