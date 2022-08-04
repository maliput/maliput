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
#include "maliput/geometry_base/kd_tree_reorganization.h"

#include "maliput/math/kd_tree.h"

namespace maliput {
namespace geometry_base {

class KDTreeReorganization::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)
  Impl(std::deque<MaliputPoint>&& points)
      : kd_tree_(std::make_unique<math::KDTree<MaliputPoint, 3>>(std::move(points))) {}

  ~Impl() = default;

  std::unique_ptr<maliput::math::KDTree<MaliputPoint, 3>> kd_tree_;
};

KDTreeReorganization::KDTreeReorganization(std::deque<MaliputPoint>&& points)
    : SpacialReorganization(), pimpl_(std::make_unique<Impl>(std::move(points))) {}

KDTreeReorganization::~KDTreeReorganization() = default;

maliput::api::LaneId KDTreeReorganization::do_closest_lane(const maliput::math::Vector3& point) const {
  // TODO(): Once range search is available in the kdtree, this method should be deprecated and the do_closest_lanes
  // should be used instead.
  //         Given that this method should actually obtain the lanes within a tolerance radius. Afterwards, the lane
  //         could be obtained using the point being closer to the centerline.
  const auto maliput_point = pimpl_->kd_tree_->Nearest(point);
  return maliput_point.lane_id();
}

std::vector<maliput::api::LaneId> KDTreeReorganization::do_closest_lanes(const maliput::math::Vector3& point,
                                                                         double distance) const {
  // TODO
  return {};
}

}  // namespace geometry_base
}  // namespace maliput
