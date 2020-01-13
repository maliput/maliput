#include "dragway/road_geometry.h"

#include <cmath>
#include <memory>

#include "drake/common/unused.h"
#include "drake/math/saturate.h"

#include "maliput/common/logger.h"
#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"

#include "dragway/branch_point.h"
#include "dragway/junction.h"

using std::make_unique;

namespace maliput {
namespace dragway {

RoadGeometry::RoadGeometry(const api::RoadGeometryId& id, int num_lanes, double length, double lane_width,
                           double shoulder_width, double maximum_height, double linear_tolerance,
                           double angular_tolerance)
    : id_(id),
      linear_tolerance_(linear_tolerance),
      angular_tolerance_(angular_tolerance),
      // Dragway is completely flat and featureless, so the scale-length might
      // as well be any value that characterizes its overall size.
      // TODO(maddog@tri.global)  If this code is ever re-arranged in a way that
      //                          makes it possible to query the segment for its
      //                          `road_width`, then use the max of length and
      //                          width.
      scale_length_(length),
      junction_(this, num_lanes, length, lane_width, shoulder_width, maximum_height) {
  MALIPUT_DEMAND(length > 0);
  MALIPUT_DEMAND(lane_width > 0);
  MALIPUT_DEMAND(shoulder_width >= 0);
  MALIPUT_DEMAND(maximum_height >= 0);
  MALIPUT_DEMAND(linear_tolerance >= 0);
  MALIPUT_DEMAND(angular_tolerance >= 0);

  id_index_.WalkAndAddAll(this);
}

const api::Junction* RoadGeometry::do_junction(int index) const {
  MALIPUT_DEMAND(index < num_junctions());
  return &junction_;
}

int RoadGeometry::do_num_branch_points() const {
  // There is only one BranchPoint per lane. Thus, return the number of lanes.
  return junction_.segment(0)->num_lanes();
}

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const {
  MALIPUT_DEMAND(index < num_branch_points());
  // The same BranchPoint is at the start versus end of a Lane, thus it doesn't
  // matter whether the start or finish BranchPoint is returned.
  return junction_.segment(0)->lane(index)->GetBranchPoint(api::LaneEnd::kStart);
}

bool RoadGeometry::IsGeoPositionOnDragway(const api::GeoPosition& geo_pos) const {
  const Lane* lane = dynamic_cast<const Lane*>(junction_.segment(0)->lane(0));
  MALIPUT_DEMAND(lane != nullptr);
  const double length = lane->length();
  const api::RBounds lane_segment_bounds = lane->segment_bounds(0 /* s */);
  const double min_y = lane->y_offset() + lane_segment_bounds.min();
  const double max_y = lane->y_offset() + lane_segment_bounds.max();

  if (geo_pos.x() < 0 || geo_pos.x() > length || geo_pos.y() > max_y || geo_pos.y() < min_y) {
    maliput::log()->trace(
        "dragway::RoadGeometry::IsGeoPositionOnDragway(): The provided geo_pos "
        "({}, {}) is not on the dragway (length = {}, min_y = {}, max_y = {}).",
        geo_pos.x(), geo_pos.y(), length, min_y, max_y);
    return false;
  } else {
    return true;
  }
}

int RoadGeometry::GetLaneIndex(const api::GeoPosition& geo_pos) const {
  MALIPUT_DEMAND(IsGeoPositionOnDragway(geo_pos));
  bool lane_found{false};
  int result{0};
  for (int i = 0; !lane_found && i < junction_.segment(0)->num_lanes(); ++i) {
    const Lane* lane = dynamic_cast<const Lane*>(junction_.segment(0)->lane(i));
    MALIPUT_DEMAND(lane != nullptr);
    if (geo_pos.y() <= lane->y_offset() + lane->lane_bounds(0).max()) {
      result = i;
      lane_found = true;
    }

    // Checks whether `geo_pos` is on the right shoulder. If it is, save the
    // index of the right-most lane in `result`.
    if (lane->to_right() == nullptr) {
      if (geo_pos.y() <= lane->y_offset() + lane->lane_bounds(0).min() &&
          geo_pos.y() >= lane->y_offset() + lane->segment_bounds(0).min()) {
        result = i;
        lane_found = true;
      }
    }

    // Checks whether `geo_pos` is on the left shoulder. If it is, save the
    // index of the left-most lane in `result`.
    if (lane->to_left() == nullptr) {
      if (geo_pos.y() >= lane->y_offset() + lane->lane_bounds(0).max() &&
          geo_pos.y() <= lane->y_offset() + lane->segment_bounds(0).max()) {
        result = i;
        lane_found = true;
      }
    }
  }
  if (!lane_found) {
    MALIPUT_THROW_MESSAGE(
        "dragway::RoadGeometry::GetLaneIndex: Failed to "
        "find lane for geo_pos (" +
        std::to_string(geo_pos.x()) + ", " + std::to_string(geo_pos.y()) + ").");
  }
  return result;
}

api::RoadPositionResult RoadGeometry::DoToRoadPosition(const api::GeoPosition& geo_pos,
                                                       const std::optional<api::RoadPosition>& hint) const {
  drake::unused(hint);

  // Computes the dragway's (x,y) segment surface coordinates.
  MALIPUT_DEMAND(junction_.num_segments() > 0);
  const api::Segment* segment = junction_.segment(0);
  MALIPUT_DEMAND(segment != nullptr);
  MALIPUT_DEMAND(segment->num_lanes() > 0);
  const Lane* lane = dynamic_cast<const Lane*>(segment->lane(0));
  MALIPUT_DEMAND(lane != nullptr);
  const double length = lane->length();
  const api::RBounds lane_segment_bounds = lane->segment_bounds(0 /* s */);
  const double min_y = lane->y_offset() + lane_segment_bounds.min();
  const double max_y = lane->y_offset() + lane_segment_bounds.max();
  const double min_x = 0;
  const double max_x = length;
  const double min_z = lane->elevation_bounds(0, 0).min();
  const double max_z = lane->elevation_bounds(0, 0).max();

  /*
      A figure of a typical dragway is shown below. The minimum and maximum
      values of the dragway' segment surface are demarcated.

                            X
              Y = max_y     ^      Y = min_y
                            :
                  |         :         |
                  |         :         |
          --------+---------+---------+---------  X = max_x
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .  The  .   . |
                  | .   . Dragway   . |
                  | .   .   :   .   . |
                  | .   .   :   .   . |
                  | .   .   :   .   . |
     Y <----------+---------o---------+---------  X = min_x
                  |         :         |
                  |         :         |
                            :
                            V

      The (x, y) coordinate of the closest point is basically the (x, y)
      coordinates of the provide `geo_pos` clamped by the minimum and maximum
      values of of the dragway' segment surface. This can be encoded as
      follows.
  */
  api::GeoPosition closest_position;
  closest_position.set_x(drake::math::saturate(geo_pos.x(), min_x, max_x));
  closest_position.set_y(drake::math::saturate(geo_pos.y(), min_y, max_y));
  closest_position.set_z(drake::math::saturate(geo_pos.z(), min_z, max_z));

  const int closest_lane_index = GetLaneIndex(closest_position);
  const Lane* closest_lane = dynamic_cast<const Lane*>(junction_.segment(0)->lane(closest_lane_index));
  MALIPUT_DEMAND(closest_lane != nullptr);
  const api::LanePosition closest_lane_position(closest_position.x() /* s */,
                                                closest_position.y() - closest_lane->y_offset() /* r */,
                                                closest_position.z() /* h */);
  return api::RoadPositionResult{api::RoadPosition(closest_lane, closest_lane_position), closest_position,
                                 (geo_pos.xyz() - closest_position.xyz()).norm()};
}

std::vector<api::RoadPositionResult> RoadGeometry::DoFindRoadPositions(const api::GeoPosition& geo_position,
                                                                       double radius) const {
  return maliput::geometry_base::BruteForceFindRoadPositionsStrategy(this, geo_position, radius);
}

}  // namespace dragway
}  // namespace maliput
