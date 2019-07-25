#include "maliput/test_utilities/mock_geometry.h"

#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"


namespace maliput {
namespace geometry_base {
namespace test {


api::RoadPosition MockRoadGeometry::DoToRoadPosition(
    const api::GeoPosition&, const api::RoadPosition*,
    api::GeoPosition*, double*) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

std::vector<api::RoadPositionResult>
MockRoadGeometry::DoFindRoadPositions(const api::GeoPosition&, double) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

double MockLane::do_length() const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::RBounds MockLane::do_lane_bounds(double) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::RBounds MockLane::do_driveable_bounds(double) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::HBounds MockLane::do_elevation_bounds(double, double) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::GeoPosition MockLane::DoToGeoPosition(const api::LanePosition&) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::Rotation MockLane::DoGetOrientation(const api::LanePosition&) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::LanePosition MockLane::DoEvalMotionDerivatives(
    const api::LanePosition&, const api::IsoLaneVelocity&) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::LanePosition MockLane::DoToLanePosition(
    const api::GeoPosition&, api::GeoPosition*, double*) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}


}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
