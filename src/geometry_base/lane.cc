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
#include "maliput/geometry_base/lane.h"

#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/geometry_base/branch_point.h"

namespace maliput {
namespace geometry_base {

void Lane::AttachToSegment(common::Passkey<Segment>, const api::Segment* segment, int index) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(segment != nullptr);
  MALIPUT_THROW_UNLESS(index >= 0);
  // Preconditions
  MALIPUT_THROW_UNLESS(segment_ == nullptr);
  MALIPUT_THROW_UNLESS(index_ == -1);

  segment_ = segment;
  index_ = index;
}

void Lane::SetStartBranchPoint(common::Passkey<BranchPoint>, BranchPoint* branch_point) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(branch_point != nullptr);
  // Preconditions
  MALIPUT_THROW_UNLESS(start_branch_point_ == nullptr);

  start_branch_point_ = branch_point;
}

void Lane::SetFinishBranchPoint(common::Passkey<BranchPoint>, BranchPoint* branch_point) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(branch_point != nullptr);
  // Preconditions
  MALIPUT_THROW_UNLESS(finish_branch_point_ == nullptr);

  finish_branch_point_ = branch_point;
}

api::LaneId Lane::do_id() const { return id_; }

const api::Segment* Lane::do_segment() const { return segment_; }

int Lane::do_index() const { return index_; }

const api::Lane* Lane::do_to_left() const {
  if (index_ == (segment_->num_lanes() - 1)) {
    return nullptr;
  } else {
    return segment_->lane(index_ + 1);
  }
}

const api::Lane* Lane::do_to_right() const {
  if (index_ == 0) {
    return nullptr;
  } else {
    return segment_->lane(index_ - 1);
  }
}

const api::BranchPoint* Lane::DoGetBranchPoint(const api::LaneEnd::Which which_end) const {
  switch (which_end) {
    case api::LaneEnd::kStart: {
      return start_branch_point_;
    }
    case api::LaneEnd::kFinish: {
      return finish_branch_point_;
    }
  }
  MALIPUT_ABORT_MESSAGE("which_end is neither LaneEnd::kStart nor LaneEnd::kFinish.");
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetOngoingBranches({this, which_end});
}

std::optional<api::LaneEnd> Lane::DoGetDefaultBranch(api::LaneEnd::Which which_end) const {
  return GetBranchPoint(which_end)->GetDefaultBranch({this, which_end});
}

api::InertialPosition Lane::DoToInertialPosition(const api::LanePosition& lane_pos) const {
  const math::Vector3 backend_pos = DoToBackendPosition(lane_pos);
  return api::InertialPosition::FromXyz(
      backend_pos + segment()->junction()->road_geometry()->inertial_to_backend_frame_translation());
}

math::Vector3 Lane::DoToBackendPosition(const api::LanePosition& lane_pos) const {
  MALIPUT_THROW_MESSAGE(
      "Unimplemented method. Please check the documentation of "
      "maliput::geometry_base::Lane::DoToInertialPosition().");
}

api::LanePositionResult Lane::DoToLanePosition(const api::InertialPosition& inertial_pos) const {
  const math::Vector3 inertial_to_backend_frame_translation =
      segment()->junction()->road_geometry()->inertial_to_backend_frame_translation();
  const math::Vector3 backend_pos = inertial_pos.xyz() - inertial_to_backend_frame_translation;

  api::LanePosition lane_pos{};
  math::Vector3 nearest_backend_pos{};
  double distance{};
  DoToLanePositionBackend(backend_pos, &lane_pos, &nearest_backend_pos, &distance);
  return {lane_pos, api::InertialPosition::FromXyz(nearest_backend_pos + inertial_to_backend_frame_translation),
          distance};
}

void Lane::DoToLanePositionBackend(const math::Vector3& backend_pos, api::LanePosition* lane_position,
                                   math::Vector3* nearest_backend_pos, double* distance) const {
  MALIPUT_THROW_MESSAGE(
      "Unimplemented method. Please check the documentation of "
      "maliput::geometry_base::Lane::DoToLanePosition().");
}

}  // namespace geometry_base
}  // namespace maliput
