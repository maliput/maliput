#include "dragway/junction.h"

#include "dragway/road_geometry.h"
#include "dragway/segment.h"

namespace maliput {
namespace dragway {

Junction::Junction(RoadGeometry* road_geometry,
    int num_lanes,
    double length,
    double lane_width,
    double shoulder_width,
    double maximum_height)
  : id_("Dragway Junction"),
    road_geometry_(road_geometry),
    segment_(this, num_lanes, length, lane_width, shoulder_width,
             maximum_height) {
  MALIPUT_DEMAND(road_geometry != nullptr);
}

const api::RoadGeometry* Junction::do_road_geometry() const {
  return road_geometry_;
}

}  // namespace dragway
}  // namespace maliput
