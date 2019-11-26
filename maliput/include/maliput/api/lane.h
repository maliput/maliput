#pragma once

#include <memory>
#include <string>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/symbolic.h"

#include "maliput/api/lane_data.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {

class BranchPoint;
class Segment;
class LaneEndSet;

/// Persistent identifier for a Lane element.
using LaneId = TypeSpecificIdentifier<class Lane>;

/// A Lane represents a lane of travel in a road network.  A Lane defines
/// a curvilinear coordinate system covering the road surface, with a
/// longitudinal 's' coordinate that expresses the arc-length along a
/// central reference curve.  The reference curve nominally represents
/// an ideal travel trajectory along the Lane.
///
/// Lanes are grouped by Segment.  All Lanes belonging to a Segment
/// represent the same road surface, but with different coordinate
/// parameterizations (e.g., each Lane has its own reference curve).
class Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

  virtual ~Lane() = default;

  /// Returns the persistent identifier.
  LaneId id() const { return do_id(); }

  /// Returns the Segment to which this Lane belongs.
  const Segment* segment() const { return do_segment(); }

  /// Returns the index of this Lane within the Segment which owns it.
  int index() const { return do_index(); }

  /// Returns a pointer to the adjacent Lane to the left of this Lane.
  ///
  /// Left is considered the +r direction with regards to the (s,r,h) frame,
  /// e.g., "to the left along the +s direction".
  ///
  /// @returns nullptr iff parent Segment has no Lane to the left.
  const Lane* to_left() const { return do_to_left(); }

  /// Returns a pointer to the adjacent Lane to the right of this Lane.
  ///
  /// Right is considered the -r direction with regards to the (s,r,h) frame,
  /// e.g., "to the right along the +s direction".
  ///
  /// @returns nullptr iff parent Segment has no Lane to the right.
  const Lane* to_right() const { return do_to_right(); }

  /// Returns the arc-length of the Lane along its reference curve.
  ///
  /// The value of length() is also the maximum s-coordinate for this Lane;
  /// i.e., the domain of s is [0, length()].
  double length() const { return do_length(); }

  /// Returns the nominal lateral (r) bounds for the lane as a function of s.
  ///
  /// These are the lateral bounds for a position that is considered to be
  /// "staying in the lane".
  ///
  /// @see segment_bounds(double s) that defines the whole surface.
  RBounds lane_bounds(double s) const { return do_lane_bounds(s); }

  /// Returns the lateral segment (r) bounds of the lane as a function of s.
  ///
  /// These are the lateral bounds for a position that is considered to be
  /// "on segment", reflecting the physical extent of the surface of the
  /// lane's segment.
  ///
  /// @see lane_bounds(double s) that defines what's considered to be "staying
  /// in the lane".
  RBounds segment_bounds(double s) const { return do_segment_bounds(s); }

  /// Returns the elevation (`h`) bounds of the lane as a function of `(s, r)`.
  ///
  /// These are the elevation bounds for a position that is considered to be
  /// within the Lane's volume modeled by the RoadGeometry.
  ///
  /// `s` is within [0, `length()`] of this Lane and `r` is within
  /// `lane_bounds(s)`.
  HBounds elevation_bounds(double s, double r) const { return do_elevation_bounds(s, r); }

  /// Returns the GeoPosition corresponding to the given LanePosition.
  ///
  /// @pre The s component of @p lane_pos must be in domain [0, Lane::length()].
  /// @pre The r component of @p lane_pos must be in domain [Rmin, Rmax]
  ///      derived from Lane::segment_bounds().
  //
  // TODO(jadecastro): Generalize `Lane::ToGeoPosition` (and possibly others)
  // with another member function `Lane::ToGeoPositionT<T>()`.
  GeoPosition ToGeoPosition(const LanePosition& lane_pos) const { return DoToGeoPosition(lane_pos); }

  // TODO(jadecastro): Apply this implementation in all the subclasses of
  // `api::Lane`.
  /// Generalization of ToGeoPosition to arbitrary scalar types, where the
  /// structures `LanePositionT<T>` and `GeoPositionT<T>` are used in place of
  /// `LanePosition` and `GeoPosition`, respectively.
  ///
  /// When the arguments are of type drake::AutoDiffXd, the return value is a
  /// GeoPositionT<drake::AutoDiffXd> containing the same partial derivatives as those
  /// appearing in lane_pos.  The provided lane_pos must be internally
  /// consistent; the s, r, and h variables must have derivatives of equal size,
  /// and where the i-th derivative of one variable is taken with respect to the
  /// same quantity as the i-th derviative of another variable.
  ///
  /// Instantiated templates for the following kinds of T's are provided:
  ///
  /// - double
  /// - drake::AutoDiffXd
  /// - drake::symbolic::Expression
  ///
  /// They are already available to link against in the containing library.
  ///
  /// @note This is an experimental API that is not necessarily implemented in
  /// all back-end implementations.
  template <typename T>
  GeoPositionT<T> ToGeoPositionT(const LanePositionT<T>& lane_pos) const;

  /// Determines the LanePosition corresponding to GeoPosition @p geo_pos.
  ///
  /// This method guarantees that its result satisfies the condition that
  /// `ToGeoPosition(result.lane_position)` is within `linear_tolerance()`
  ///  of `result.nearest_position`.
  LanePositionResult ToLanePosition(const GeoPosition& geo_pos) const { return DoToLanePosition(geo_pos); }

  /// Generalization of ToLanePosition to arbitrary scalar types, where the
  /// structures `LanePositionResultT<T>` and `GeoPositionT<T>` are used in
  /// place of `LanePositionResult` and `GeoPosition`, respectively.
  ///
  /// When the arguments are of type drake::AutoDiffXd, the return value is a
  /// LanePositionResultT<drake::AutoDiffXd> containing the same partial
  /// derivatives as those appearing in `geo_pos`. The provided geo_pos must
  /// be internally consistent; the x, y, and z variables must have derivatives
  /// of equal size, and where the i-th derivative of one variable is taken
  /// with respect to the same quantity as the i-th derivative of another
  /// variable.
  /// Result nearest position and distance will also contain @p geo_pos's
  /// partial derivatives.
  ///
  /// Instantiated templates for the following kinds of T's are provided:
  ///
  /// - double
  /// - drake::AutoDiffXd
  /// - drake::symbolic::Expression
  ///
  /// They are already available to link against in the containing library.
  ///
  /// @note This is an experimental API that is not necessarily implemented in
  /// all back-end implementations.
  //
  // TODO(jadecastro): Consider having the client enforce the geo_pos drake::AutoDiffXd
  // coherency contract rather than api::Lane.  Reevaluate this once an AutoDiff
  // consumer for api::ToLanePositionT() exists.
  //
  // TODO(jadecastro): Apply this implementation in all the subclasses of
  // `api::Lane`.
  template <typename T>
  LanePositionResultT<T> ToLanePositionT(const GeoPositionT<T>& geo_pos) const;

  // TODO(maddog@tri.global) Method to convert LanePosition to that of
  //                         another Lane.  (Should assert that both
  //                         lanes belong to same Segment.)  It should look
  //                         something like this:
  //           LanePosition ToOtherLane(const LanePosition& in_this_lane,
  //                                    const Lane* other_lane) const;

  /// Returns the rotation which expresses the orientation of the
  /// `Lane`-frame basis at @p lane_pos with respect to the
  /// world frame basis.
  Rotation GetOrientation(const LanePosition& lane_pos) const { return DoGetOrientation(lane_pos); }

  /// Computes derivatives of LanePosition given a velocity vector @p velocity.
  /// @p velocity is a isometric velocity vector oriented in the `Lane`-frame
  /// at @p position.
  ///
  /// @returns `Lane`-frame derivatives packed into a LanePosition struct.
  LanePosition EvalMotionDerivatives(const LanePosition& position, const IsoLaneVelocity& velocity) const {
    return DoEvalMotionDerivatives(position, velocity);
  }

  // TODO(maddog@tri.global)  Design/implement this.
  // void EvalSurfaceDerivatives(...) const { return do_(); }

  /// Returns the lane's BranchPoint for the end specified by @p which_end.
  const BranchPoint* GetBranchPoint(const LaneEnd::Which which_end) const { return DoGetBranchPoint(which_end); }

  /// Returns the set of LaneEnd's which connect with this lane on the
  /// same side of the BranchPoint at @p which_end.  At a minimum,
  /// this set will include this Lane.
  const LaneEndSet* GetConfluentBranches(const LaneEnd::Which which_end) const {
    return DoGetConfluentBranches(which_end);
  }

  /// Returns the set of LaneEnd's which continue onward from this lane at the
  /// BranchPoint at @p which_end.
  const LaneEndSet* GetOngoingBranches(const LaneEnd::Which which_end) const { return DoGetOngoingBranches(which_end); }

  /// Returns the default ongoing LaneEnd connected at @p which_end,
  /// or drake::nullopt if no default branch has been established at @p which_end.
  drake::optional<LaneEnd> GetDefaultBranch(const LaneEnd::Which which_end) const {
    return DoGetDefaultBranch(which_end);
  }

  bool Contains(const LanePosition& lane_position);

 protected:
  Lane() = default;

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraints/invariants of the
  /// corresponding public methods.
  ///@{
  virtual LaneId do_id() const = 0;

  virtual const Segment* do_segment() const = 0;

  virtual int do_index() const = 0;

  virtual const Lane* do_to_left() const = 0;

  virtual const Lane* do_to_right() const = 0;

  virtual double do_length() const = 0;

  virtual RBounds do_lane_bounds(double s) const = 0;

  virtual RBounds do_segment_bounds(double s) const = 0;

  virtual HBounds do_elevation_bounds(double s, double r) const = 0;

  virtual GeoPosition DoToGeoPosition(const LanePosition& lane_pos) const = 0;

  virtual LanePositionResult DoToLanePosition(const GeoPosition& geo_pos) const = 0;

  virtual Rotation DoGetOrientation(const LanePosition& lane_pos) const = 0;

  virtual LanePosition DoEvalMotionDerivatives(const LanePosition& position, const IsoLaneVelocity& velocity) const = 0;

  virtual const BranchPoint* DoGetBranchPoint(const LaneEnd::Which which_end) const = 0;

  virtual const LaneEndSet* DoGetConfluentBranches(const LaneEnd::Which which_end) const = 0;

  virtual const LaneEndSet* DoGetOngoingBranches(const LaneEnd::Which which_end) const = 0;

  virtual drake::optional<LaneEnd> DoGetDefaultBranch(const LaneEnd::Which which_end) const = 0;

  // drake::AutoDiffXd overload of DoToGeoPosition().
  virtual GeoPositionT<drake::AutoDiffXd> DoToGeoPositionAutoDiff(const LanePositionT<drake::AutoDiffXd>&) const {
    MALIPUT_THROW_MESSAGE(
        "DoToGeoPosition has been instantiated with drake::AutoDiffXd arguments, "
        "but a Lane backend has not overridden its drake::AutoDiffXd specialization.");
  }

  // drake::AutoDiffXd overload of DoToLanePosition().
  virtual LanePositionResultT<drake::AutoDiffXd> DoToLanePositionAutoDiff(
      const GeoPositionT<drake::AutoDiffXd>&) const {
    MALIPUT_THROW_MESSAGE(
        "DoToLanePosition has been instantiated with drake::AutoDiffXd arguments, "
        "but a Lane backend has not overridden its drake::AutoDiffXd specialization.");
  }

  // drake::symbolic::Expression overload of DoToGeoPosition().
  virtual GeoPositionT<drake::symbolic::Expression> DoToGeoPositionSymbolic(
      const LanePositionT<drake::symbolic::Expression>&) const {
    MALIPUT_THROW_MESSAGE(
        "DoToGeoPosition has been instantiated with drake::symbolic::Expression "
        "arguments, but a Lane backend has not overridden its "
        "drake::symbolic::Expression specialization.");
  }

  // drake::symbolic::Expression overload of DoToLanePosition().
  virtual LanePositionResultT<drake::symbolic::Expression> DoToLanePositionSymbolic(
      const GeoPositionT<drake::symbolic::Expression>&) const {
    MALIPUT_THROW_MESSAGE(
        "DoToLanePosition has been instantiated with drake::symbolic::Expression "
        "arguments, but a Lane backend has not overridden its "
        "drake::symbolic::Expression specialization.");
  }

  // TODO(jadecastro): Template the entire `api::Lane` class to prevent explicit
  // virtual functions for each member function.
  ///@}
};

}  // namespace api
}  // namespace maliput
