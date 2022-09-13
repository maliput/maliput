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

KDTreeStrategy::KDTreeStrategy(const api::RoadGeometry* rg) : StrategyBase(rg) {
  const auto lanes = GetRoadGeometry()->ById().GetLanes();
  std::deque<MaliputPoint> points;
  for (const auto& lane : lanes) {
    const auto lane_length = lane.second->length();
    for (double s = 0; s <= lane_length; s += 0.1) {
      const auto lane_bounds = lane.second->lane_bounds(s);
      for (double r = lane_bounds.min(); r <= lane_bounds.max(); r += 0.1) {
        const auto inertial_pos = lane.second->ToInertialPosition({s, r, 0. /* h */}).xyz();
        const MaliputPoint point{{inertial_pos.x(), inertial_pos.y(), inertial_pos.z()}, lane.first};
        points.push_back(point);
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
  return this->ClosestLane(inertial_position.xyz());
}

std::vector<api::RoadPositionResult> KDTreeStrategy::DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                                         double radius) const {
  const auto lane_ids = this->ClosestLanes(inertial_position.xyz(), radius);
  std::vector<api::RoadPositionResult> road_positions;
  const api::RoadGeometry* rg = GetRoadGeometry();
  for (const auto& current_lane : lane_ids) {
    const auto lane = rg->ById().GetLane(current_lane);
    const auto lane_position = lane->ToLanePosition(inertial_position);
    if (lane_position.distance <= radius) {
      api::RoadPositionResult road_position{
          {lane, lane_position.lane_position}, lane_position.nearest_position, lane_position.distance};
      road_positions.push_back(road_position);
    }
  }
  return road_positions;
}

api::RoadPositionResult KDTreeStrategy::ClosestLane(const math::Vector3& point) const {
  // Given that this method should actually obtain the lanes within a tolerance radius. Afterwards, the lane
  // could be obtained using the point being closer to the centerline.
  const api::InertialPosition& inertial_position{point.x(), point.y(), point.z()};
  const MaliputPoint maliput_point = kd_tree_->nearest_point(point);
  const double radius = (point - maliput_point).norm() + 0.2;
  const std::set<api::LaneId> lane_ids = ClosestLanes(point, radius);
  const api::RoadGeometry* rg = GetRoadGeometry();

  const api::Lane* lane_result = rg->ById().GetLane(maliput_point.lane_id().value());
  api::LanePositionResult lane_position_result = lane_result->ToLanePosition(inertial_position);
  api::RoadPositionResult road_position_result{{lane_result, lane_position_result.lane_position},
                                               lane_position_result.nearest_position,
                                               lane_position_result.distance};

  for (const auto& current_lane_id : lane_ids) {
    const api::Lane* current_lane = rg->ById().GetLane(current_lane_id);
    const api::LanePositionResult current_lane_position = current_lane->ToLanePosition(inertial_position);
    const api::RoadPositionResult current_road_position{{current_lane, current_lane_position.lane_position},
                                                        current_lane_position.nearest_position,
                                                        current_lane_position.distance};

    if (IsNewRoadPositionResultCloser(current_road_position, road_position_result)) {
      road_position_result = current_road_position;
    }
  }
  return road_position_result;
}

std::set<api::LaneId> KDTreeStrategy::ClosestLanes(const math::Vector3& point, double distance) const {
  const math::Vector3 min_corner{point.x() - distance, point.y() - distance, point.z() - distance};
  const math::Vector3 max_corner{point.x() + distance, point.y() + distance, point.z() + distance};
  const math::AxisAlignedBox search_region{min_corner, max_corner};
  std::deque<const MaliputPoint*> maliput_points = kd_tree_->RangeSearch(search_region);
  std::set<api::LaneId> maliput_lanes;
  /*std::transform(maliput_points.begin(), maliput_points.end(),
                   maliput_lanes.begin(),
                   [](auto point) {return point->lane_id().value();});*/
  for (const auto& current_point : maliput_points) {
    maliput_lanes.insert(current_point->lane_id().value());
  }
  return maliput_lanes;
}
}  // namespace geometry_base
}  // namespace maliput
