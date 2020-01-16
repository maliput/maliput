#pragma once

#include <memory>
#include <optional>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

#include "maliput/api/branch_point.h"
#include "maliput/api/lane.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_abort.h"

#include "multilane/cubic_polynomial.h"
#include "multilane/road_curve.h"

namespace maliput {
namespace multilane {

class BranchPoint;

typedef drake::Vector2<double> V2;
typedef drake::Vector3<double> V3;

/// Base class for the multilane implementation of api::Lane.
class Lane : public api::Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  /// Constructs a Lane.
  ///
  /// @param id the ID
  /// @param segment the Segment to which this Lane will belong, which must
  ///        remain valid for the lifetime of this class
  /// @param index Lane's index to identify it when querying parent @p segment.
  ///        It must be positive.
  /// @param lane_bounds nominal bounds of the lane, uniform along the entire
  ///        reference path, which must be a subset of @p segment_bounds
  /// @param segment_bounds segment bounds of the lane, uniform along the
  ///        entire reference path
  /// @param elevation_bounds elevation bounds of the lane, uniform along the
  ///        entire segment surface
  /// @param road_curve The trajectory of the Lane over parent @p segment's
  ///        surface.
  /// @param r0 The lateral displacement with respect to the @p road_curve's
  ///        reference curve.
  ///
  /// @note The override Lane::ToLanePosition() is currently restricted to
  /// lanes in which superelevation and elevation change are both zero.
  Lane(const api::LaneId& id, const api::Segment* segment, int index, const api::RBounds& lane_bounds,
       const api::RBounds& segment_bounds, const api::HBounds& elevation_bounds, const RoadCurve* road_curve, double r0)
      : id_(id),
        segment_(segment),
        index_(index),
        lane_bounds_(lane_bounds),
        segment_bounds_(segment_bounds),
        elevation_bounds_(elevation_bounds),
        road_curve_(road_curve),
        r0_(r0) {
    MALIPUT_DEMAND(index_ >= 0);
    MALIPUT_DEMAND(lane_bounds_.min() >= segment_bounds_.min());
    MALIPUT_DEMAND(lane_bounds_.max() <= segment_bounds_.max());
    MALIPUT_DEMAND(road_curve != nullptr);
    s_from_p_at_r0_ = road_curve_->OptimizeCalcSFromP(r0_);
    p_from_s_at_r0_ = road_curve_->OptimizeCalcPFromS(r0_);
    lane_length_ = s_from_p_at_r0_(1.0);
  }

  // TODO(maddog-tri)  Allow superelevation to have a center-of-rotation
  //                   which is different from r = 0.

  const CubicPolynomial& elevation() const { return road_curve_->elevation(); }

  const CubicPolynomial& superelevation() const { return road_curve_->superelevation(); }

  double r0() const { return r0_; }

  void SetStartBp(BranchPoint* bp) { start_bp_ = bp; }
  void SetEndBp(BranchPoint* bp) { end_bp_ = bp; }

  BranchPoint* start_bp() { return start_bp_; }

  BranchPoint* end_bp() { return end_bp_; }

  ~Lane() override = default;

 private:
  api::LaneId do_id() const override { return id_; }

  const api::Segment* do_segment() const override;

  int do_index() const override { return index_; }

  const api::Lane* do_to_left() const override {
    if (index_ == (segment_->num_lanes() - 1)) {
      return nullptr;
    } else {
      return segment_->lane(index_ + 1);
    }
  }

  const api::Lane* do_to_right() const override {
    if (index_ == 0) {
      return nullptr;
    } else {
      return segment_->lane(index_ - 1);
    }
  }

  const api::BranchPoint* DoGetBranchPoint(const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetConfluentBranches(const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(const api::LaneEnd::Which which_end) const override;

  std::optional<api::LaneEnd> DoGetDefaultBranch(const api::LaneEnd::Which which_end) const override;

  double do_length() const override { return lane_length_; }

  api::RBounds do_lane_bounds(double) const override { return lane_bounds_; }

  api::RBounds do_segment_bounds(double) const override { return segment_bounds_; }

  api::HBounds do_elevation_bounds(double, double) const override { return elevation_bounds_; }

  api::GeoPosition DoToGeoPosition(const api::LanePosition& lane_pos) const override;

  api::Rotation DoGetOrientation(const api::LanePosition& lane_pos) const override;

  api::LanePosition DoEvalMotionDerivatives(const api::LanePosition& position,
                                            const api::IsoLaneVelocity& velocity) const override;

  api::LanePositionResult DoToLanePosition(const api::GeoPosition& geo_position) const override;

  const api::LaneId id_;
  const api::Segment* segment_{};
  const int index_{};
  BranchPoint* start_bp_{};
  BranchPoint* end_bp_{};

  const api::RBounds lane_bounds_;
  const api::RBounds segment_bounds_;
  const api::HBounds elevation_bounds_;
  const RoadCurve* road_curve_{};
  const double r0_;

  std::function<double(double)> s_from_p_at_r0_;
  std::function<double(double)> p_from_s_at_r0_;
  double lane_length_;
};

}  // namespace multilane
}  // namespace maliput
