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

#include <algorithm>
#include <cstdlib>

#include "maliput/math/kd_tree.h"

namespace maliput {
namespace geometry_base {

KDTreeStrategy::KDTreeStrategy(const api::RoadGeometry* rg, const double sampling_step)
    : StrategyBase(rg), sampling_step_(sampling_step) {
  const auto lanes = get_road_geometry()->ById().GetLanes();
  std::deque<MaliputPoint> points;
  for (const auto& lane : lanes) {
    const auto lane_length = lane.second->length();
    for (double s = 0; s <= lane_length; s += sampling_step_) {
      const auto lane_bounds = lane.second->lane_bounds(s);
      for (double r = lane_bounds.min(); r <= lane_bounds.max(); r += sampling_step_) {
        const auto inertial_pos = lane.second->ToInertialPosition({s, r, 0. /* h */}).xyz();
        points.push_back(MaliputPoint{{inertial_pos.x(), inertial_pos.y(), inertial_pos.z()}, lane.second});
      }
    }
  }
  kd_tree_ = std::make_unique<math::KDTree3D<MaliputPoint>>(std::move(points));
}

api::RoadPositionResult KDTreeStrategy::DoToRoadPosition(const api::InertialPosition& inertial_position,
                                                         const std::optional<api::RoadPosition>& hint) const {
  if (hint.has_value()) {
    MALIPUT_THROW_UNLESS(hint->lane != nullptr);
    const api::LanePositionResult lane_pos = hint->lane->ToLanePosition(inertial_position);
    return {{hint->lane, lane_pos.lane_position}, lane_pos.nearest_position, lane_pos.distance};
  }
  return this->ClosestLane(inertial_position);
}

std::vector<api::RoadPositionResult> KDTreeStrategy::DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                                         double radius) const {
  const auto closest_lanes = this->ClosestLanes(inertial_position, radius);
  std::vector<api::RoadPositionResult> road_positions;
  for (const auto& lane : closest_lanes) {
    MALIPUT_THROW_UNLESS(lane != nullptr);
    const auto lane_position = lane->ToLanePosition(inertial_position);
    if (lane_position.distance <= radius) {
      road_positions.push_back(
          {{lane, lane_position.lane_position}, lane_position.nearest_position, lane_position.distance});
    }
  }
  return road_positions;
}

api::RoadPositionResult KDTreeStrategy::ClosestLane(const api::InertialPosition& point) const {
  // Obtains the closest point in the kd-tree to the given point.
  const MaliputPoint maliput_point = kd_tree_->nearest_point(MaliputPoint{point.xyz()});
  // As the kd-tree is built with a sampling step, the closest point may not be the closest lane.
  // Therefore, we search for the closest lane in a axis-aligned box whose half edge length is the distance between the
  // nearest point and the given point plus twice the sampling_step_.
  const double half_edge_length = (point.xyz() - maliput_point).norm() + 2. * sampling_step_;
  const std::deque<const api::Lane*> closest_lanes = ClosestLanes(point, half_edge_length);

  // Once we have the lanes in the region, we search for the closest lane relying on the lane's ToLanePosition method.
  MALIPUT_THROW_UNLESS(maliput_point.get_lane().has_value());
  const api::Lane* lane_result = maliput_point.get_lane().value();
  api::LanePositionResult lane_position_result = lane_result->ToLanePosition(point);
  api::RoadPositionResult road_position_result{{lane_result, lane_position_result.lane_position},
                                               lane_position_result.nearest_position,
                                               lane_position_result.distance};

  for (const auto& lane : closest_lanes) {
    MALIPUT_THROW_UNLESS(lane != nullptr);
    const api::LanePositionResult lane_position = lane->ToLanePosition(point);
    const api::RoadPositionResult road_position{
        {lane, lane_position.lane_position}, lane_position.nearest_position, lane_position.distance};

    if (IsNewRoadPositionResultCloser(road_position, road_position_result)) {
      road_position_result = road_position;
    }
  }
  return road_position_result;
}

std::deque<const api::Lane*> KDTreeStrategy::ClosestLanes(const api::InertialPosition& point,
                                                          double half_edge_length) const {
  const math::Vector3 min_corner{point.x() - half_edge_length, point.y() - half_edge_length,
                                 point.z() - half_edge_length};
  const math::Vector3 max_corner{point.x() + half_edge_length, point.y() + half_edge_length,
                                 point.z() + half_edge_length};
  const math::AxisAlignedBox search_region{min_corner, max_corner};
  const std::deque<const MaliputPoint*> maliput_points = kd_tree_->RangeSearch(search_region);
  std::deque<const api::Lane*> maliput_lanes(maliput_points.size());
  std::transform(maliput_points.begin(), maliput_points.end(), maliput_lanes.begin(),
                 [](const MaliputPoint* point) { return point->get_lane().value(); });
  return maliput_lanes;
}
}  // namespace geometry_base
}  // namespace maliput
