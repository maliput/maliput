// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "maliput/api/lane_data.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
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
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Lane)

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

  /// Returns the InertialPosition corresponding to the given LanePosition.
  ///
  /// @pre The s component of @p lane_pos must be in domain [0, Lane::length()].
  /// @pre The r component of @p lane_pos must be in domain [Rmin, Rmax]
  ///      derived from Lane::segment_bounds().
  InertialPosition ToInertialPosition(const LanePosition& lane_pos) const { return DoToInertialPosition(lane_pos); }

  /// Determines the LanePosition corresponding to InertialPosition @p inertial_pos.
  /// The LanePosition is expected to be contained within the lane's boundaries.
  /// @see ToSegmentPosition method.
  ///
  /// This method guarantees that its result satisfies the condition that
  /// `ToInertialPosition(result.lane_position)` is within `linear_tolerance()`
  ///  of `result.nearest_position`.
  LanePositionResult ToLanePosition(const InertialPosition& inertial_pos) const {
    return DoToLanePosition(inertial_pos);
  }

  /// Determines the LanePosition corresponding to InertialPosition @p inertial_pos.
  /// The LanePosition is expected to be contained within the segment's boundaries.
  /// @see ToLanePosition method.
  ///
  /// This method guarantees that its result satisfies the condition that
  /// `ToInertialPosition(result.lane_position)` is within `linear_tolerance()`
  ///  of `result.nearest_position`.
  LanePositionResult ToSegmentPosition(const InertialPosition& inertial_pos) const {
    return DoToSegmentPosition(inertial_pos);
  }

  // TODO(maddog@tri.global) Method to convert LanePosition to that of
  //                         another Lane.  (Should assert that both
  //                         lanes belong to same Segment.)  It should look
  //                         something like this:
  //           LanePosition ToOtherLane(const LanePosition& in_this_lane,
  //                                    const Lane* other_lane) const;

  /// Returns the rotation which expresses the orientation of the
  /// `Lane`-frame basis at @p lane_pos with respect to the
  /// `Inertial`-frame basis.
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
  /// or std::nullopt if no default branch has been established at @p which_end.
  std::optional<LaneEnd> GetDefaultBranch(const LaneEnd::Which which_end) const {
    return DoGetDefaultBranch(which_end);
  }

  /// Returns if this lane contains @p lane_position.
  bool Contains(const LanePosition& lane_position) const;

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

  virtual InertialPosition DoToInertialPosition(const LanePosition& lane_pos) const = 0;

  virtual LanePositionResult DoToLanePosition(const InertialPosition& inertial_pos) const = 0;

  virtual LanePositionResult DoToSegmentPosition(const InertialPosition& inertial_pos) const = 0;

  virtual Rotation DoGetOrientation(const LanePosition& lane_pos) const = 0;

  virtual LanePosition DoEvalMotionDerivatives(const LanePosition& position, const IsoLaneVelocity& velocity) const = 0;

  virtual const BranchPoint* DoGetBranchPoint(const LaneEnd::Which which_end) const = 0;

  virtual const LaneEndSet* DoGetConfluentBranches(const LaneEnd::Which which_end) const = 0;

  virtual const LaneEndSet* DoGetOngoingBranches(const LaneEnd::Which which_end) const = 0;

  virtual std::optional<LaneEnd> DoGetDefaultBranch(const LaneEnd::Which which_end) const = 0;
  ///@}
};

}  // namespace api
}  // namespace maliput
