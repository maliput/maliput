// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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
#include <cstdlib>

#include "maliput/geometry_base/kd_tree_reorganization.h"
#include "maliput/math/kd_tree.h"

namespace maliput {
namespace geometry_base {

class KDTreeReorganization::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)
  Impl(const api::RoadGeometry* rg, std::deque<MaliputPoint>&& points)
      : rg_(rg), kd_tree_(std::make_unique<math::KDTree3D<MaliputPoint>>(std::move(points))) {}

  ~Impl() = default;
  const api::RoadGeometry* rg_;
  std::unique_ptr<maliput::math::KDTree3D<MaliputPoint>> kd_tree_;
};

KDTreeReorganization::KDTreeReorganization(const api::RoadGeometry* rg, std::deque<MaliputPoint>&& points)
    : SpacialReorganization(), pimpl_(std::make_unique<Impl>(rg, std::move(points))) {}

KDTreeReorganization::~KDTreeReorganization() = default;

maliput::api::LaneId KDTreeReorganization::do_closest_lane(const maliput::math::Vector3& point) const {
  // Given that this method should actually obtain the lanes within a tolerance radius. Afterwards, the lane
  // could be obtained using the point being closer to the centerline.
  const api::InertialPosition& inertial_position{point.x(), point.y(), point.z()};
  const auto maliput_point = pimpl_->kd_tree_->Nearest(point);
  double radius = std::sqrt(pow(point.x() - maliput_point.x(), 2) + pow(point.y() - maliput_point.y(), 2) +
                                  pow(point.z() - maliput_point.z(), 2));
  const auto lane_ids = do_closest_lanes(point, radius);

  auto lane_id_result = maliput_point.lane_id();
  auto lane_position_result = pimpl_->rg_->ById().GetLane(lane_id_result)->ToLanePosition(inertial_position);
  double min_distance = lane_position_result.distance;

  for (const auto &current_lane: lane_ids) {
    const auto current_lane_position = pimpl_->rg_->ById().GetLane(maliput_point.lane_id())->ToLanePosition(inertial_position);
    if(current_lane_position.distance < min_distance) {
      min_distance = current_lane_position.distance;
      lane_position_result = current_lane_position;
      lane_id_result = current_lane;
    } else if (current_lane_position.distance == min_distance) {
      if (std::abs(current_lane_position.lane_position.r()) < std::abs(lane_position_result.lane_position.r())){
        min_distance = current_lane_position.distance;
        lane_position_result = current_lane_position;
        lane_id_result = current_lane;
      }
    }
  }
  return lane_id_result;
}

std::set<maliput::api::LaneId> KDTreeReorganization::do_closest_lanes(const maliput::math::Vector3& point,
                                                                         double distance) const {
  const maliput::math::Vector3 min_corner{point.x()-distance, point.y()-distance, point.z()-distance};
  const maliput::math::Vector3 max_corner{point.x()+distance, point.y()+distance, point.z()+distance};
  const math::AxisAlignedBox& search_region{min_corner, max_corner};
  const auto maliput_points = pimpl_->kd_tree_->RangeSearch(search_region);
  std::set<maliput::api::LaneId> maliput_lanes;
  for (int i = 0; i < maliput_points.size(); i++) {
      maliput_lanes.insert(maliput_points[i]->lane_id());
  }
  return maliput_lanes;
}

}  // namespace geometry_base
}  // namespace maliput
