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

#include "maliput/api/branch_point.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/passkey.h"

namespace maliput {
namespace geometry_base {

class BranchPoint;
class Segment;

/// geometry_base's implementation of api::Lane.
class Lane : public api::Lane {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Lane);

  /// Constructs a Lane.
  ///
  /// @param id the ID of the Lane
  ///
  /// The Lane is not fully initialized until it is added to a Segment
  /// and each end is added to (one or two) BranchPoints.
  ///
  /// The Lane's index will be assigned when it is added to a Segment.
  explicit Lane(const api::LaneId& id) : id_(id) {}

  /// Returns a mutable pointer to the BranchPoint at the start end,
  /// or nullptr if the start end hasn't been added to a BranchPoint yet.
  BranchPoint* mutable_start_branch_point() { return start_branch_point_; }

  /// Returns a mutable pointer to the BranchPoint at the finish end,
  /// or nullptr if the finish end hasn't been added to a BranchPoint yet.
  BranchPoint* mutable_finish_branch_point() { return finish_branch_point_; }

  // Notifies Lane of its parent Segment.
  // This may only be called, once, by a Segment.
  //
  // @param segment  the parent Segment
  // @param index  index of this Lane within `segment`
  //
  // @pre `segment` is non-null.
  // @pre `index` is non-negative.
  // @pre Parent Segment and the index have not already been set.
  void AttachToSegment(common::Passkey<Segment>, const api::Segment* segment, int index);

  // Notifies Lane of the BranchPoint for its "start" end.
  // This may only be called, once, by a BranchPoint.
  //
  // @param branch_point  the BranchPoint
  //
  // @pre `branch_point` is non-null.
  // @pre The "start" BranchPoint has not already been set.
  void SetStartBranchPoint(common::Passkey<BranchPoint>, BranchPoint* branch_point);

  // Notifies Lane of the BranchPoint for its "finish" end.
  // This may only be called, once, by a BranchPoint.
  //
  // @param branch_point  the BranchPoint
  //
  // @pre `branch_point` is non-null.
  // @pre The "finish" BranchPoint has not already been set.
  void SetFinishBranchPoint(common::Passkey<BranchPoint>, BranchPoint* branch_point);

  ~Lane() override = default;

 private:
  api::LaneId do_id() const override;

  const api::Segment* do_segment() const override;

  int do_index() const override;

  const api::Lane* do_to_left() const override;

  const api::Lane* do_to_right() const override;

  const api::BranchPoint* DoGetBranchPoint(const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetConfluentBranches(const api::LaneEnd::Which which_end) const override;

  const api::LaneEndSet* DoGetOngoingBranches(const api::LaneEnd::Which which_end) const override;

  std::optional<api::LaneEnd> DoGetDefaultBranch(const api::LaneEnd::Which which_end) const override;

  // @{
  // Maps @p lane_pos into the Inertial Frame.
  //
  // @details Forwards the call to DoToBackendPosition(lane_pos) and translates
  //          the result to account for api::RoadGeometry::inertial_to_backend_frame_translation().
  //
  // @note Because of performance constraints, backends must decide whether to
  //       exclusively override DoToInertialPosition() or DoToBackendPosition().
  //       When overriding DoToInertialPosition(), a better performance is
  //       expected than the generic transform applied here.
  virtual api::InertialPosition DoToInertialPosition(const api::LanePosition& lane_pos) const override;

  // Maps @p lane_pos into the Backend Frame.
  //
  // @note This method implementation @throws maliput::common::assertion_error
  //       as it should only be called when it is overridden by a derived class.
  //       @see DoToInertialPosition() docstring to understand when to override
  //       each method accordingly.
  //
  // @return The mapped @p lane_pos into the Backend Frame.
  virtual math::Vector3 DoToBackendPosition(const api::LanePosition& lane_pos) const;
  // @}

  // @{
  // Maps @p inertial_pos (measured in the Inertial Frame) into this Lane Frame.
  //
  // @details Translates @p inertial_pos with api::RoadGeometry::internal_inertial_frame_translation()
  //          and calls DoToLanePositionBackend(). The returned values are
  //          used to build an api::LanePositionResult. Note that
  //          `nearest_backend_pos` is converted back to the Inertial Frame
  //          before returning the final result.
  //
  // @note Because of performance constraints, backends must decide whether to
  //       exclusively override DoToLanePosition() or DoToLanePositionBackend().
  //       When overriding DoToLanePosition(), a better performance is
  //       expected than the generic transform applied here.
  virtual api::LanePositionResult DoToLanePosition(const api::InertialPosition& inertial_pos) const override;

  // Maps @p backend_pos (measured in the Backend Frame) into this Lane Frame.
  //
  // @note This method implementation @throws maliput::common::assertion_error
  //       as it should only be called when it is overridden by a derived class.
  //       @see DoToLanePosition() docstring to understand when to override each
  //       method accordingly.
  //
  // @param backend_pos The Backend Frame coordinate to map into this Lane
  //        Frame.
  // @param lane_pos The candidate LanePosition within the Lane' segment-
  //        bounds which is closest closest to @p backend_pos. It must not be
  //        nullptr.
  // @param nearest_backend_pos The position that exactly corresponds to
  //        @p lane_pos. It must not be nullptr.
  // @param distance The Cartesian distance between `nearest_position` and the
  //        Inertial Frame position supplied to Lane::ToLanePosition() and then
  //        converted into the Backend Frame. It must not be nullptr.
  //
  // @throws When any of @p backend_pos, @p lane_pos, or @p distance is
  //         nullptr.
  //
  // @return The @p lane_pos, @p nearest_backend_pos and @p distance.
  virtual void DoToLanePositionBackend(const math::Vector3& backend_pos, api::LanePosition* lane_pos,
                                       math::Vector3* nearest_backend_pos, double* distance) const;
  // @}

  const api::LaneId id_;
  const api::Segment* segment_{};
  int index_{-1};
  BranchPoint* start_branch_point_{};
  BranchPoint* finish_branch_point_{};
};

}  // namespace geometry_base
}  // namespace maliput
