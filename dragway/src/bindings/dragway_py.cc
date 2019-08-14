#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

#include "dragway/road_geometry.h"
#include "maliput/api/road_geometry.h"

namespace maliput {
namespace dragway {
namespace bindings {

namespace py = pybind11;

using std::make_unique;
using std::unique_ptr;

PYBIND11_MODULE(dragway, m) {
  py::module::import("maliput");

  py::class_<RoadGeometry, api::RoadGeometry>(m, "RoadGeometry");

  m.def("create_dragway",
        [](api::RoadGeometryId road_id, int num_lanes, double length, double lane_width, double shoulder_width,
           double maximum_height, double linear_tolerance, double angular_tolerance) {
          return make_unique<RoadGeometry>(road_id, num_lanes, length, lane_width, shoulder_width, maximum_height,
                                           linear_tolerance, angular_tolerance);
        },
        py::arg("road_id"), py::arg("num_lanes"), py::arg("length"), py::arg("lane_width"), py::arg("shoulder_width"),
        py::arg("maximum_height"), py::arg("linear_tolerance"), py::arg("angular_tolerance"));
}

}  // namespace bindings
}  // namespace dragway
}  // namespace maliput
