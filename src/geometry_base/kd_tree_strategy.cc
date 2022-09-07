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

#include "maliput/geometry_base/kd_tree_strategy.h"

#include <cstdlib>

#include "maliput/math/kd_tree.h"

namespace maliput {
namespace geometry_base {

api::RoadPositionResult KDTreeStrategy::DoToRoadPosition(const api::InertialPosition& inertial_position,
                                                         const std::optional<api::RoadPosition>& hint) const {
  if (hint.has_value()) {
    MALIPUT_THROW_UNLESS(hint->lane != nullptr);
    const maliput::api::LanePositionResult lane_pos = hint->lane->ToLanePosition(inertial_position);
    return {{hint->lane, lane_pos.lane_position}, lane_pos.nearest_position, lane_pos.distance};
  }
  const auto lane_id = this->ClosestLane(inertial_position.xyz());
  const auto lane = rg_->ById().GetLane(lane_id);
  const auto lane_position_result = lane->ToLanePosition(inertial_position);
  return {
      {lane, lane_position_result.lane_position}, lane_position_result.nearest_position, lane_position_result.distance};
}

std::vector<api::RoadPositionResult> KDTreeStrategy::DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                                         double radius) const {
  const auto lane_ids = this->ClosestLanes(inertial_position.xyz(), radius);
  std::vector<api::RoadPositionResult> road_positions;
  for (const auto& current_lane : lane_ids) {
    const auto lane = rg_->ById().GetLane(current_lane);
    const auto lane_position = lane->ToLanePosition(inertial_position);
    if (lane_position.distance <= radius) {
      api::RoadPositionResult road_position{
          {lane, lane_position.lane_position}, lane_position.nearest_position, lane_position.distance};
      road_positions.push_back(road_position);
    }
  }
  return road_positions;
}

void KDTreeStrategy::Init() {
  const auto lanes = rg_->ById().GetLanes();

  std::deque<maliput::geometry_base::MaliputPoint> points;
  for (const auto& lane : lanes) {
    const auto lane_length = lane.second->length();
    for (double s = 0; s <= lane_length; s += 0.1) {
      const auto lane_bounds = lane.second->lane_bounds(s);
      for (double r = lane_bounds.min(); r <= lane_bounds.max(); r += 0.1) {
        const auto inertial_pos = lane.second->ToInertialPosition({s, r, 0. /* h */}).xyz();
        const maliput::geometry_base::MaliputPoint point{{inertial_pos.x(), inertial_pos.y(), inertial_pos.z()},
                                                         lane.first};
        points.push_back(point);
      }
    }
  }
  kd_tree_ = (std::make_unique<math::KDTree3D<MaliputPoint>>(std::move(points)));
}

maliput::api::LaneId KDTreeStrategy::do_closest_lane(const maliput::math::Vector3& point) const {
  // Given that this method should actually obtain the lanes within a tolerance radius. Afterwards, the lane
  // could be obtained using the point being closer to the centerline.
  const api::InertialPosition& inertial_position{point.x(), point.y(), point.z()};
  const auto maliput_point = kd_tree_->Nearest(point);
  double radius = std::sqrt(pow(point.x() - maliput_point.x(), 2) + pow(point.y() - maliput_point.y(), 2) +
                            pow(point.z() - maliput_point.z(), 2)) +
                  0.02;
  const auto lane_ids = do_closest_lanes(point, radius);

  auto lane_id_result = maliput_point.lane_id();
  auto lane_position_result = rg_->ById().GetLane(lane_id_result)->ToLanePosition(inertial_position);
  double min_distance = lane_position_result.distance;
  const static double KLineraTolerance{1e-12};  // [m]

  for (const auto& current_lane : lane_ids) {
    const auto current_lane_position = rg_->ById().GetLane(maliput_point.lane_id())->ToLanePosition(inertial_position);
    if (current_lane_position.distance - min_distance > KLineraTolerance) {
      min_distance = current_lane_position.distance;
      lane_position_result = current_lane_position;
      lane_id_result = current_lane;
      continue;
    }
    if (current_lane_position.distance - min_distance <= -KLineraTolerance) {
      continue;
    }
    if (std::abs(current_lane_position.lane_position.r()) - std::abs(lane_position_result.lane_position.r()) <
        -KLineraTolerance) {
      min_distance = current_lane_position.distance;
      lane_position_result = current_lane_position;
      lane_id_result = current_lane;
      continue;
    }
    if (std::abs(current_lane_position.lane_position.r()) - std::abs(lane_position_result.lane_position.r()) >
        KLineraTolerance) {
      continue;
    }
  }
  return lane_id_result;
}

std::set<maliput::api::LaneId> KDTreeStrategy::do_closest_lanes(const maliput::math::Vector3& point,
                                                                double distance) const {
  const maliput::math::Vector3 min_corner{point.x() - distance, point.y() - distance, point.z() - distance};
  const maliput::math::Vector3 max_corner{point.x() + distance, point.y() + distance, point.z() + distance};
  const math::AxisAlignedBox& search_region{min_corner, max_corner};
  const auto maliput_points = kd_tree_->RangeSearch(search_region);
  std::set<maliput::api::LaneId> maliput_lanes;
  for (const auto& current_point : maliput_points) {
    maliput_lanes.insert(current_point->lane_id());
  }
  return maliput_lanes;
}
}  // namespace geometry_base
}  // namespace maliput
