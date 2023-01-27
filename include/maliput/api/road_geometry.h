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

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "maliput/api/branch_point.h"
#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/segment.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace api {

class BranchPoint;
class Junction;

/// Persistent identifier for a RoadGeometry element.
using RoadGeometryId = TypeSpecificIdentifier<class RoadGeometry>;

// TODO(maddog@tri.global)  This entire API should be templated on a
//                          scalar type T.
/// Abstract API for the geometry of a road network, including both
/// the network topology and the geometry of its embedding in 3-space.
class RoadGeometry {
 public:
  class IdIndex;

  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadGeometry)

  virtual ~RoadGeometry() = default;

  /// Returns the persistent identifier.
  ///
  // TODO(maddog@tri.global)  Tie id into a tiling mechanism?
  RoadGeometryId id() const { return do_id(); }

  // TODO(maddog@tri.global) Proper iterators over junctions and branch-points?

  /// Returns the number of Junctions in the RoadGeometry.
  ///
  /// Return value is non-negative.
  int num_junctions() const { return do_num_junctions(); }

  /// Returns the Junction indexed by @p index.
  ///
  /// @pre @p index must be >= 0 and < num_junctions().
  const Junction* junction(int index) const { return do_junction(index); }

  /// Returns the number of BranchPoints in the RoadGeometry.
  ///
  /// Return value is non-negative.
  int num_branch_points() const { return do_num_branch_points(); }

  /// Returns the BranchPoint indexed by @p index.
  ///
  /// @pre @p index must be >= 0 and < num_branch_points().
  const BranchPoint* branch_point(int index) const { return do_branch_point(index); }

  /// Accesses the IdIndex interface, which allows getting elements of
  /// the RoadGeometry's object graph by their unique id's.
  const IdIndex& ById() const { return DoById(); }

  /// Determines the RoadPosition corresponding to InertialPosition @p inertial_position.
  ///
  /// If @p hint is provided, its value is used to help determine the result.
  ///
  /// Returns a RoadPositionResult. Its RoadPosition is the point in the
  /// RoadGeometry's manifold which is, in the `Inertial`-frame, closest to
  /// @p inertial_position. Its InertialPosition is the `Inertial`-frame equivalent of the
  /// RoadPosition and its distance is the Cartesian distance from
  /// @p inertial_position to the nearest point.
  ///
  /// This method guarantees that its result satisfies the condition that
  /// `result.lane->ToInertialPosition(result.pos)` is within `linear_tolerance()`
  /// of the returned InertialPosition.
  ///
  /// The map from RoadGeometry to the `Inertial`-frame is not onto (as a bounded
  /// RoadGeometry cannot completely cover the unbounded Cartesian universe).
  /// If @p inertial_position does represent a point contained within the volume
  /// of the RoadGeometry, then result distance is guaranteed to be less
  /// than or equal to `linear_tolerance()`.
  ///
  /// The map from RoadGeometry to `Inertial`-frame is not necessarily one-to-one.
  /// Different `(s,r,h)` coordinates from different Lanes, potentially from
  /// different Segments, may map to the same `(x,y,z)` `Inertial`-frame location.
  ///
  /// If @p inertial_position is contained within the volumes of multiple Segments,
  /// then ToRoadPosition() will choose a Segment which yields the minimum
  /// height `h` value in the result.  If the chosen Segment has multiple
  /// Lanes, then ToRoadPosition() will choose a Lane which contains
  /// @p inertial_position within its `lane_bounds()` if possible, and if that is
  /// still ambiguous, it will further select a Lane which minimizes the
  /// absolute value of the lateral `r` coordinate in the result.
  // TODO(maddog@tri.global)  Establish what effect `hint` has on the outcome.
  //                          Two notions:
  //                          1)  the hint helps to bootstrap a search;
  //                          2)  the hint helps to choose between multiple
  //                              equally valid solutions.
  //                          The general idea:  if one knows the RoadPosition
  //                          of an entity some small dT in the past, then one
  //                          might expect an updated RoadPosition which is
  //                          nearby (e.g., on the same Lane).
  RoadPositionResult ToRoadPosition(const InertialPosition& inertial_position,
                                    const std::optional<RoadPosition>& hint = std::nullopt) const;

  /// Obtains all RoadPositions within @p radius of @p inertial_position. Only Lanes
  /// whose segment regions include points that are within @p radius of
  /// @p inertial_position are included in the search. For each of these Lanes,
  /// include the RoadPosition or RoadPositions with the minimum distance to
  /// @p inertial_position in the returned result.
  ///
  /// @param inertial_position The inertial position to convert into one or more
  ///        RoadPositions.
  /// @param radius The maximum distance from @p inertial_position to search. It must
  ///        not be negative.
  /// @return A vector of RoadPositionResults representing the possible
  ///         RoadPositions. When @p radius is zero, the vector contains results
  ///         with distance parameter being less or equal to linear_tolerance.
  ///         When @p radius is infinity, the query should return the closest
  ///         point for each lane.
  ///
  /// @throws maliput::common::assertion_error When @p radius is negative.
  ///
  /// Note that derivative implementations may choose to violate the above
  /// semantics for performance reasons. See docstrings of derivative
  /// implementations for details.
  std::vector<RoadPositionResult> FindRoadPositions(const InertialPosition& inertial_position, double radius) const;

  /// Returns the tolerance guaranteed for linear measurements (positions).
  double linear_tolerance() const { return do_linear_tolerance(); }

  /// Returns the tolerance guaranteed for angular measurements (orientations).
  double angular_tolerance() const { return do_angular_tolerance(); }

  // TODO(maddog@tri.global) Needs a precise mathematical definition.
  /// Returns the characteristic scale length expressed by this RoadGeometry.
  double scale_length() const { return do_scale_length(); }

  /// Verifies certain invariants guaranteed by the API.
  ///
  /// Returns a vector of strings describing violations of invariants.
  /// Return value with size() == 0 indicates success.
  std::vector<std::string> CheckInvariants() const;

  /// Samples `lane_s_route` at `path_length_sampling_rate` and converts those
  /// LanePositions into InertialPositions.
  ///
  /// When `path_length_sampling_rate` is smaller than linear_tolerance, linear_tolerance
  /// will be used instead. When `path_length_sampling_rate` is bigger than total
  /// `lane_s_route` length (accumulated length of all LaneSRoute::ranges()) the minimum
  /// will be considered and two samples are taken.
  /// When total `lane_s_route`'s length is not an integral multiple of `path_length_sampling_rate`,
  /// the last sampling step will be the remaining distance long only.
  ///
  /// @param lane_s_route A lane route.
  /// @param path_length_sampling_rate The `s` coordinate sampling rate to sample `lane_s_route`. It must be positive.
  /// @returns A vector of InertialPositions which result of mapping to the inertial frame
  ///                the samples of LanePositions.
  /// @throws maliput::assertion_error When `path_length_sampling_rate` is not positive.
  /// @throws maliput::assertion_error When any LaneSRange in `lane_s_route.ranges()` refers to
  ///                an unknown Lane.
  std::vector<InertialPosition> SampleAheadWaypoints(const LaneSRoute& lane_s_route,
                                                     double path_length_sampling_rate) const;

  /// The Backend Frame is an inertial frame similar to the Inertial Frame that
  /// differ one from another by an isometric transformation. This method
  /// returns the translation vector between both frames.
  ///
  /// For an explanation on the two different frames, see the explanation in
  /// maliput_design.h.
  ///
  /// @return maliput's Inertial Frame to Backend Frame translation vector.
  math::Vector3 inertial_to_backend_frame_translation() const;

  // TODO(#400): Add RollPitchYaw inertial_to_backend_frame_rotation() const.

 protected:
  RoadGeometry() = default;

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraints/invariants of the
  /// corresponding public methods.
  ///@{
  virtual RoadGeometryId do_id() const = 0;

  virtual int do_num_junctions() const = 0;

  virtual const Junction* do_junction(int index) const = 0;

  virtual int do_num_branch_points() const = 0;

  virtual const BranchPoint* do_branch_point(int index) const = 0;

  virtual const IdIndex& DoById() const = 0;

  virtual RoadPositionResult DoToRoadPosition(const InertialPosition& inertial_position,
                                              const std::optional<RoadPosition>& hint) const = 0;

  virtual std::vector<RoadPositionResult> DoFindRoadPositions(const InertialPosition& inertial_position,
                                                              double radius) const = 0;

  virtual double do_linear_tolerance() const = 0;

  virtual double do_angular_tolerance() const = 0;

  virtual double do_scale_length() const = 0;

  virtual std::vector<InertialPosition> DoSampleAheadWaypoints(const LaneSRoute&,
                                                               double path_length_sampling_rate) const;

  virtual math::Vector3 do_inertial_to_backend_frame_translation() const = 0;
  ///@}
};

/// Abstract interface for a collection of methods which allow accessing
/// objects in a RoadGeometry's object graph (Lanes, Segments, Junctions,
/// BranchPoints) by their unique id's.
class RoadGeometry::IdIndex {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(IdIndex)
  virtual ~IdIndex() = default;

  /// Returns the Lane identified by @p id, or `nullptr` if @p id is unknown.
  const Lane* GetLane(const LaneId& id) const;

  // Returns all of the Lane instances.
  const std::unordered_map<LaneId, const Lane*>& GetLanes() const;

  /// Returns the Segment identified by @p id, or `nullptr` if @p id is
  /// unknown.
  const Segment* GetSegment(const SegmentId& id) const;

  /// Returns the Junction identified by @p id, or `nullptr` if @p id is
  /// unknown.
  const Junction* GetJunction(const JunctionId& id) const;

  /// Returns the BranchPoint identified by @p id, or `nullptr` if @p id is
  /// unknown.
  const BranchPoint* GetBranchPoint(const BranchPointId& id) const;

 protected:
  IdIndex() = default;

 private:
  virtual const Lane* DoGetLane(const LaneId& id) const = 0;
  virtual const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const = 0;
  virtual const Segment* DoGetSegment(const SegmentId& id) const = 0;
  virtual const Junction* DoGetJunction(const JunctionId& id) const = 0;
  virtual const BranchPoint* DoGetBranchPoint(const BranchPointId& id) const = 0;
};

}  // namespace api
}  // namespace maliput
