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
#include "maliput/geometry_base/road_geometry.h"

#include <utility>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace geometry_base {

void RoadGeometry::AddJunctionPrivate(std::unique_ptr<Junction> junction) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(junction.get() != nullptr);
  junctions_.emplace_back(std::move(junction));
  Junction* const raw_junction = junctions_.back().get();
  // clang-format off
  raw_junction->AttachToRoadGeometry({}, this, [this](auto segment) { id_index_.AddSegment(segment); },
                                     [this](auto lane) { id_index_.AddLane(lane); });
  // clang-format on
  id_index_.AddJunction(raw_junction);
}

void RoadGeometry::AddBranchPointPrivate(std::unique_ptr<BranchPoint> branch_point) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(branch_point.get() != nullptr);
  branch_points_.emplace_back(std::move(branch_point));
  BranchPoint* const raw_branch_point = branch_points_.back().get();
  raw_branch_point->AttachToRoadGeometry({}, this);
  id_index_.AddBranchPoint(raw_branch_point);
}

const api::Junction* RoadGeometry::do_junction(int index) const { return junctions_.at(index).get(); }

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const { return branch_points_.at(index).get(); }

/*
void RoadGeometry::SpacialReorganization(const SpacialReorganization::Type& type) {
  // Sample the surface using Lane::ToInertialPosition.
  const auto lanes = this->ById().GetLanes();

  std::deque<maliput::geometry_base::MaliputPoint> points;
  for (const auto& lane : lanes) {
    const auto lane_length = lane.second->length();
    for (double s = 0; s <= lane_length; s += 0.1) {
      const auto lane_bounds = lane.second->lane_bounds(s);
      for (double r = lane_bounds.min(); r <= lane_bounds.max(); r += 0.1) {
        const auto inertial_pos = lane.second->ToInertialPosition({s, r, h}).xyz(); //return h to 0.
        const maliput::geometry_base::MaliputPoint point{{inertial_pos.x(), inertial_pos.y(), inertial_pos.z()},
                                                         lane.first};
        points.push_back(point);
      }
    }
  }
  switch (type) {
    case SpacialReorganization::Type::kKDTree:
      spacial_reorganization_ = std::make_unique<maliput::geometry_base::KDTreeReorganization>(this, std::move(points));
      break;
    default:
      MALIPUT_THROW_MESSAGE("Unsupported spacial reorganization type");
      break;
  }
}
*/

api::RoadPositionResult RoadGeometry::DoToRoadPosition(const api::InertialPosition& inertial_position,
                                                       const std::optional<api::RoadPosition>& hint) const {
  MALIPUT_THROW_UNLESS(strategy_ != nullptr);
  return strategy_->ToRoadPosition(inertial_position, hint);
  /*
  if (hint.has_value()) {
    MALIPUT_THROW_UNLESS(hint->lane != nullptr);
    const maliput::api::LanePositionResult lane_pos = hint->lane->ToLanePosition(inertial_position);
    return {{hint->lane, lane_pos.lane_position}, lane_pos.nearest_position, lane_pos.distance};
  }
  const auto lane_id = spacial_reorganization_->ClosestLane(inertial_position.xyz());
  const auto lane = this->ById().GetLane(lane_id);
  const auto lane_position_result = lane->ToLanePosition(inertial_position);
  return {
      {lane, lane_position_result.lane_position}, lane_position_result.nearest_position, lane_position_result.distance};
  */
}

std::vector<api::RoadPositionResult> RoadGeometry::DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                                       double radius) const {
  MALIPUT_THROW_UNLESS(strategy_ != nullptr);
  return strategy_->FindRoadPositions(inertial_position, radius);
  /*
  const auto lane_ids = spacial_reorganization_->ClosestLanes(inertial_position.xyz(), radius);
  std::vector<api::RoadPositionResult> road_positions;
  for (const auto& current_lane : lane_ids) {
    const auto lane = this->ById().GetLane(current_lane);
    const auto lane_position = lane->ToLanePosition(inertial_position);
    if (lane_position.distance <= radius) {
      api::RoadPositionResult road_position{
          {lane, lane_position.lane_position}, lane_position.nearest_position, lane_position.distance};
      road_positions.push_back(road_position);
    }
  }
  return road_positions;
  */
}

}  // namespace geometry_base
}  // namespace maliput
