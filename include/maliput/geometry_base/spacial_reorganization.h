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

#include <deque>

#include "maliput/api/lane.h"

namespace maliput {
namespace geometry_base {

class SpacialReorganization {
 public:
  enum class Type {
    kKDTree,
  };

  virtual ~SpacialReorganization() = default;

  maliput::api::LaneId ClosestLane(const maliput::math::Vector3& point) const { return do_closest_lane(point); }

  std::vector<maliput::api::LaneId> ClosestLanes(const maliput::math::Vector3& point, double distance) const {
    return do_closest_lanes(point, distance);
  }

 protected:
  SpacialReorganization() = default;

 private:
  virtual maliput::api::LaneId do_closest_lane(const maliput::math::Vector3& point) const = 0;
  virtual std::vector<maliput::api::LaneId> do_closest_lanes(const maliput::math::Vector3& point,
                                                             double distance) const = 0;
};

}  // namespace geometry_base
}  // namespace maliput
