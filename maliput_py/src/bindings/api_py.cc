#include "pybind11/pybind11.h"

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/road_network.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace bindings {

namespace py = pybind11;

PYBIND11_MODULE(api, m) {
  // TODO(jadecastro) These bindings are work-in-progress. Expose additional
  // Maliput API features, as necessary (see #7918).

  // TODO(m-chaturvedi) Add doc when typedefs are parsed (#9599)
  py::class_<api::RoadGeometryId>(m, "RoadGeometryId")
      .def(py::init<std::string>())
      .def("string", &api::RoadGeometryId::string, py::return_value_policy::reference_internal);

  py::class_<api::InertialPosition>(m, "InertialPosition")
      .def(py::init<double, double, double>(), py::arg("x"), py::arg("y"), py::arg("z"))
      .def("xyz", &api::InertialPosition::xyz, py::return_value_policy::reference_internal);

  py::class_<api::LanePosition>(m, "LanePosition")
      .def(py::init<double, double, double>(), py::arg("s"), py::arg("r"), py::arg("h"))
      .def("srh", &api::LanePosition::srh, py::return_value_policy::reference_internal);

  py::class_<api::LanePositionResult>(m, "LanePositionResult")
      .def(py::init<>())
      .def(py::init<const api::LanePosition&, const api::InertialPosition&, double>(), py::arg("lane_position"),
           py::arg("nearest_position"), py::arg("distance"))
      .def_readwrite("lane_position", &api::LanePositionResult::lane_position)
      .def_readwrite("nearest_position", &api::LanePositionResult::nearest_position)
      .def_readwrite("distance", &api::LanePositionResult::distance);

  py::class_<api::RoadPosition> road_position(m, "RoadPosition");
  road_position  // BR
      .def(py::init<>())
      .def(py::init<const api::Lane*, const api::LanePosition&>(), py::arg("lane"), py::arg("pos"),
           // Keep alive, reference: `self` keeps `Lane*` alive.
           py::keep_alive<1, 2>())
      .def_readwrite("pos", &api::RoadPosition::pos)
      .def_property("lane", py::cpp_function([](const api::RoadPosition* self) { return self->lane; }),
                    py::cpp_function([](api::RoadPosition* self, api::Lane* value) { self->lane = value; }),
                    py::keep_alive<1, 2>());

  py::class_<api::Rotation>(m, "Rotation")
      .def("quat", &api::Rotation::quat, py::return_value_policy::reference_internal)
      .def("rpy", &api::Rotation::rpy);

  py::class_<api::RoadNetwork>(m, "RoadNetwork")
      .def("road_geometry", &api::RoadNetwork::road_geometry, py::return_value_policy::reference_internal);

  py::class_<api::RoadGeometry>(m, "RoadGeometry")
      .def("id", &api::RoadGeometry::id)
      .def("num_junctions", &api::RoadGeometry::num_junctions)
      .def("junction", &api::RoadGeometry::junction, py::return_value_policy::reference_internal)
      .def("ById", &api::RoadGeometry::ById, py::return_value_policy::reference_internal);

  py::class_<api::RoadGeometry::IdIndex>(m, "RoadGeometry.IdIndex")
      .def("GetLane",
           [](const api::RoadGeometry::IdIndex* self, const std::string& id) { return self->GetLane(api::LaneId(id)); },
           py::arg("id"), py::return_value_policy::reference_internal);

  py::class_<api::Junction>(m, "Junction")
      .def("num_segments", &api::Junction::num_segments)
      .def("segment", &api::Junction::segment, py::return_value_policy::reference_internal);

  py::class_<api::Segment>(m, "Segment")
      .def("num_lanes", &api::Segment::num_lanes)
      .def("lane", &api::Segment::lane, py::return_value_policy::reference_internal);

  // TODO(m-chaturvedi) Add Pybind11 documentation.
  py::class_<api::LaneId>(m, "LaneId")
      .def(py::init<std::string>())
      .def("string", &api::LaneId::string, py::return_value_policy::reference_internal);

  py::class_<api::Lane>(m, "Lane")
      .def("ToLanePosition", &api::Lane::ToLanePosition)
      .def("ToInertialPosition", &api::Lane::ToInertialPosition)
      .def("GetOrientation", &api::Lane::GetOrientation)
      .def("length", &api::Lane::length)
      .def("id", &api::Lane::id);
}

}  // namespace bindings
}  // namespace maliput
