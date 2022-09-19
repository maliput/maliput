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
#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "maliput/geometry_base/strategy_base.h"
#include "maliput/math/kd_tree.h"

namespace maliput {
namespace geometry_base {

/// Implements StrategyBase by reorganizing the maliput::api::Lane space into a kd-tree for
/// achieving significantly more performant queries than BruteForceStrategy.
/// The kd-tree is built in construction time by sampling the lanes, therefore
/// the RoadGeometry should be entirely built before this class instantiation.
class KDTreeStrategy final : public StrategyBase {
 public:
  KDTreeStrategy(const api::RoadGeometry* rg, double sampling_step);
  ~KDTreeStrategy() override = default;

 private:
  class MaliputPoint : public maliput::math::Vector3 {
   public:
    MaliputPoint(const Vector3& xyz) : Vector3(xyz) {}
    MaliputPoint(const Vector3& xyz, const maliput::api::LaneId& lane_id)
        : Vector3(xyz), lane_id_(std::make_optional(lane_id)) {}

    ~MaliputPoint() = default;

    std::optional<maliput::api::LaneId> lane_id() const { return lane_id_; }

   private:
    std::optional<maliput::api::LaneId> lane_id_;
  };

  api::RoadPositionResult DoToRoadPosition(const api::InertialPosition& inertial_position,
                                           const std::optional<api::RoadPosition>& hint) const override;

  std::vector<api::RoadPositionResult> DoFindRoadPositions(const api::InertialPosition& inertial_position,
                                                           double radius) const override;

  maliput::api::RoadPositionResult ClosestLane(const api::InertialPosition& point) const;

  std::set<maliput::api::LaneId> ClosestLanes(const maliput::math::Vector3& point, double distance) const;

  std::unique_ptr<maliput::math::KDTree3D<MaliputPoint>> kd_tree_;

  const double sampling_step_;
};

}  // namespace geometry_base
}  // namespace maliput
