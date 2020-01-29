#include "dragway/lane.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "drake/math/saturate.h"

#include "maliput/common/maliput_abort.h"

#include "dragway/branch_point.h"
#include "dragway/road_geometry.h"
#include "dragway/segment.h"

using std::make_unique;

namespace maliput {
namespace dragway {

Lane::Lane(const Segment* segment, const api::LaneId& id, int index, double length, double y_offset,
           const api::RBounds& lane_bounds, const api::RBounds& segment_bounds, const api::HBounds& elevation_bounds)
    : segment_(segment),
      id_(id),
      index_(index),
      length_(length),
      y_offset_(y_offset),
      lane_bounds_(lane_bounds),
      segment_bounds_(segment_bounds),
      elevation_bounds_(elevation_bounds) {
  MALIPUT_DEMAND(segment != nullptr);
  MALIPUT_DEMAND(lane_bounds_.min() >= segment_bounds_.min());
  MALIPUT_DEMAND(lane_bounds_.max() <= segment_bounds_.max());
  // TODO(liang.fok) Consider initializing this variable in the constructor's
  // initializer list so branch_point_ can be declared `const`.
  branch_point_ = make_unique<BranchPoint>(api::BranchPointId(id.string() + "_Branch_Point"), this,
                                           segment->junction()->road_geometry());
}

const api::Segment* Lane::do_segment() const { return segment_; }

void Lane::set_lane_to_left(api::Lane* lane_to_left) { lane_to_left_ = lane_to_left; }

void Lane::set_lane_to_right(api::Lane* lane_to_right) { lane_to_right_ = lane_to_right; }

const api::BranchPoint* Lane::DoGetBranchPoint(const api::LaneEnd::Which) const { return branch_point_.get(); }

const api::LaneEndSet* Lane::DoGetConfluentBranches(api::LaneEnd::Which which_end) const {
  return branch_point_->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(api::LaneEnd::Which which_end) const {
  return branch_point_->GetOngoingBranches({this, which_end});
}

std::optional<api::LaneEnd> Lane::DoGetDefaultBranch(api::LaneEnd::Which which_end) const {
  return branch_point_->GetDefaultBranch({this, which_end});
}

api::RBounds Lane::do_lane_bounds(double) const { return lane_bounds_; }

api::RBounds Lane::do_segment_bounds(double) const { return segment_bounds_; }

api::HBounds Lane::do_elevation_bounds(double, double) const { return elevation_bounds_; }

api::LanePosition Lane::DoEvalMotionDerivatives(const api::LanePosition&, const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

api::GeoPosition Lane::DoToGeoPosition(const api::LanePosition& lane_pos) const {
  return {lane_pos.s(), lane_pos.r() + Lane::y_offset(), lane_pos.h()};
}

api::Rotation Lane::DoGetOrientation(const api::LanePosition&) const {
  return api::Rotation();  // Default is Identity.
}

api::LanePositionResult Lane::DoToLanePosition(const api::GeoPosition& geo_pos) const {
  using drake::math::saturate;

  const double min_x{0.};
  const double max_x{length_};
  const double min_y{segment_bounds_.min() + y_offset_};
  const double max_y{segment_bounds_.max() + y_offset_};
  const double min_z{elevation_bounds_.min()};
  const double max_z{elevation_bounds_.max()};

  const double x = geo_pos.x();
  const double y = geo_pos.y();
  const double z = geo_pos.z();

  api::LanePositionResult result;

  result.nearest_position = {saturate(x, min_x, max_x), saturate(y, min_y, max_y), saturate(z, min_z, max_z)};

  const double distance_unsat = (geo_pos.xyz() - result.nearest_position.xyz()).norm();
  using std::max;
  result.distance = max(0., distance_unsat);

  result.lane_position = {result.nearest_position.x(), result.nearest_position.y() - y_offset_,
                          result.nearest_position.z()};

  return result;
}

}  // namespace dragway
}  // namespace maliput
