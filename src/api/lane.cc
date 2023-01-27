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
#include "maliput/api/lane.h"

#include "maliput/api/junction.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
#include "maliput/common/profiler.h"

namespace maliput {
namespace api {
namespace {

bool IsWithinRange(double x, double min, double max, double tolerance) {
  return ((min - tolerance) <= x) && (x <= (max + tolerance));
}

}  // namespace

LaneId Lane::id() const {
  MALIPUT_PROFILE_FUNC();
  return do_id();
}

const Segment* Lane::segment() const {
  MALIPUT_PROFILE_FUNC();
  return do_segment();
}

int Lane::index() const {
  MALIPUT_PROFILE_FUNC();
  return do_index();
}

const Lane* Lane::to_left() const {
  MALIPUT_PROFILE_FUNC();
  return do_to_left();
}

const Lane* Lane::to_right() const {
  MALIPUT_PROFILE_FUNC();
  return do_to_right();
}

double Lane::length() const {
  MALIPUT_PROFILE_FUNC();
  return do_length();
}

RBounds Lane::lane_bounds(double s) const {
  MALIPUT_PROFILE_FUNC();

  return do_lane_bounds(s);
}

RBounds Lane::segment_bounds(double s) const {
  MALIPUT_PROFILE_FUNC();
  return do_segment_bounds(s);
}

HBounds Lane::elevation_bounds(double s, double r) const {
  MALIPUT_PROFILE_FUNC();
  return do_elevation_bounds(s, r);
}

InertialPosition Lane::ToInertialPosition(const LanePosition& lane_pos) const {
  MALIPUT_PROFILE_FUNC();
  return DoToInertialPosition(lane_pos);
}

LanePositionResult Lane::ToLanePosition(const InertialPosition& inertial_pos) const {
  MALIPUT_PROFILE_FUNC();
  return DoToLanePosition(inertial_pos);
}

LanePositionResult Lane::ToSegmentPosition(const InertialPosition& inertial_pos) const {
  MALIPUT_PROFILE_FUNC();
  return DoToSegmentPosition(inertial_pos);
}

Rotation Lane::GetOrientation(const LanePosition& lane_pos) const {
  MALIPUT_PROFILE_FUNC();
  return DoGetOrientation(lane_pos);
}

LanePosition Lane::EvalMotionDerivatives(const LanePosition& position, const IsoLaneVelocity& velocity) const {
  MALIPUT_PROFILE_FUNC();
  return DoEvalMotionDerivatives(position, velocity);
}

const BranchPoint* Lane::GetBranchPoint(const LaneEnd::Which which_end) const {
  MALIPUT_PROFILE_FUNC();
  return DoGetBranchPoint(which_end);
}

const LaneEndSet* Lane::GetConfluentBranches(const LaneEnd::Which which_end) const {
  MALIPUT_PROFILE_FUNC();
  return DoGetConfluentBranches(which_end);
}

const LaneEndSet* Lane::GetOngoingBranches(const LaneEnd::Which which_end) const {
  MALIPUT_PROFILE_FUNC();
  return DoGetOngoingBranches(which_end);
}

std::optional<LaneEnd> Lane::GetDefaultBranch(const LaneEnd::Which which_end) const {
  MALIPUT_PROFILE_FUNC();
  return DoGetDefaultBranch(which_end);
}

bool Lane::Contains(const LanePosition& lane_position) const {
  MALIPUT_PROFILE_FUNC();
  const double s = lane_position.s();
  const double r = lane_position.r();
  const double h = lane_position.h();

  const RBounds segment_bounds = this->segment_bounds(s);
  const HBounds elevation_bounds = this->elevation_bounds(s, r);
  const double lane_length = this->length();
  const double linear_tolerance = this->segment()->junction()->road_geometry()->linear_tolerance();

  return IsWithinRange(s, 0., lane_length, linear_tolerance) &&
         IsWithinRange(r, segment_bounds.min(), segment_bounds.max(), linear_tolerance) &&
         IsWithinRange(h, elevation_bounds.min(), elevation_bounds.max(), linear_tolerance);
}

}  // namespace api
}  // namespace maliput
