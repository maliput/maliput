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
#include "maliput/geometry_base/branch_point.h"

#include "maliput/common/maliput_throw.h"
#include "maliput/geometry_base/lane.h"

namespace maliput {
namespace geometry_base {

BranchPoint::BranchPoint(const api::BranchPointId& id) : id_(id) {}

void BranchPoint::AttachToRoadGeometry(common::Passkey<RoadGeometry>, const api::RoadGeometry* road_geometry) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  // Preconditions
  MALIPUT_THROW_UNLESS(road_geometry_ == nullptr);
  road_geometry_ = road_geometry;
}

void BranchPoint::AddBranch(Lane* lane, api::LaneEnd::Which end, LaneEndSet* side) {
  // Parameter checks
  // TODO(maddog@tri.global)  This invariant should be part of api::LaneEnd.
  MALIPUT_THROW_UNLESS(lane != nullptr);
  MALIPUT_THROW_UNLESS((side == &a_side_) || (side == &b_side_));
  // Preconditions
  const api::LaneEnd lane_end{lane, end};
  MALIPUT_THROW_UNLESS(confluent_branches_.count(lane_end) == 0);
  MALIPUT_THROW_UNLESS(ongoing_branches_.count(lane_end) == 0);
  LaneEndSet* const other_side = (side == &a_side_) ? &b_side_ : &a_side_;
  side->Add(lane_end);
  confluent_branches_[lane_end] = side;
  ongoing_branches_[lane_end] = other_side;
  switch (end) {
    case api::LaneEnd::kStart: {
      lane->SetStartBranchPoint({}, this);
      break;
    }
    case api::LaneEnd::kFinish: {
      lane->SetFinishBranchPoint({}, this);
      break;
    }
  }
}

void BranchPoint::SetDefault(const api::LaneEnd& lane_end, const api::LaneEnd& default_branch) {
  // Parameter checks
  const auto& le_ongoing = ongoing_branches_.find(lane_end);
  const auto& db_confluent = confluent_branches_.find(default_branch);
  // Verify that lane_end belongs to this BranchPoint.
  MALIPUT_THROW_UNLESS(le_ongoing != ongoing_branches_.end());
  // Verify that default_branch belongs to this BranchPoint.
  MALIPUT_THROW_UNLESS(db_confluent != confluent_branches_.end());
  // Verify that default_branch is an ongoing lane for lane_end.
  MALIPUT_THROW_UNLESS(db_confluent->second == le_ongoing->second);

  defaults_[lane_end] = default_branch;
}

api::BranchPointId BranchPoint::do_id() const { return id_; }

const api::RoadGeometry* BranchPoint::do_road_geometry() const { return road_geometry_; }

const api::LaneEndSet* BranchPoint::DoGetConfluentBranches(const api::LaneEnd& end) const {
  return confluent_branches_.at(end);
}

const api::LaneEndSet* BranchPoint::DoGetOngoingBranches(const api::LaneEnd& end) const {
  return ongoing_branches_.at(end);
}

std::optional<api::LaneEnd> BranchPoint::DoGetDefaultBranch(const api::LaneEnd& end) const {
  auto default_it = defaults_.find(end);
  if (default_it == defaults_.end()) {
    return std::nullopt;
  }
  return default_it->second;
}

const api::LaneEndSet* BranchPoint::DoGetASide() const { return &a_side_; }

const api::LaneEndSet* BranchPoint::DoGetBSide() const { return &b_side_; }

}  // namespace geometry_base
}  // namespace maliput
