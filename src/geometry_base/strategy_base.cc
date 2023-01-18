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

#include "maliput/geometry_base/strategy_base.h"

#include <cstdlib>

namespace maliput {
namespace geometry_base {

bool IsNewRoadPositionResultCloser(const maliput::api::RoadPositionResult& new_road_position_result,
                                   const maliput::api::RoadPositionResult& road_position_result) {
  const double delta = new_road_position_result.distance - road_position_result.distance;
  const bool different_segment = road_position_result.road_position.lane->segment()->id() !=
                                 new_road_position_result.road_position.lane->segment()->id();

  // When lanes belong to the same segment is expected that the distance value is almost equal so we can't use the
  // distance as main condition. When lanes don't belong to the same segment we can use the distance as main
  // condition.
  if (different_segment) {
    if (delta < -1e-12) {
      return true;
    }
    if (delta >= 1e-12) {
      return false;
    }
  }

  auto is_within_lane_bounds = [](double r, const maliput::api::RBounds& lane_bounds) {
    return r >= lane_bounds.min() && r < lane_bounds.max();
  };
  // They are almost equal so it is worth checking the `r` coordinate and the
  // lane bounds.
  // When both r-coordinates fall within lane bounds or outside, the position
  // with the minimum absolute r-coordinate prevails.
  // When the new r-coordinate is within lane bounds, and the previous position
  // does not fall within lane bounds, the new result prevails.
  const maliput::api::RBounds new_lane_bounds =
      new_road_position_result.road_position.lane->lane_bounds(new_road_position_result.road_position.pos.s());
  const maliput::api::RBounds current_lane_bounds =
      road_position_result.road_position.lane->lane_bounds(road_position_result.road_position.pos.s());
  const bool is_new_within_lane_bounds =
      is_within_lane_bounds(new_road_position_result.road_position.pos.r(), new_lane_bounds);
  const bool is_current_within_lane_bounds =
      is_within_lane_bounds(road_position_result.road_position.pos.r(), current_lane_bounds);
  if ((is_new_within_lane_bounds && is_current_within_lane_bounds) ||
      (!is_new_within_lane_bounds && !is_current_within_lane_bounds)) {
    if (std::abs(new_road_position_result.road_position.pos.r()) <
        std::abs(road_position_result.road_position.pos.r())) {
      return true;
    }
  } else if (is_new_within_lane_bounds && !is_current_within_lane_bounds) {
    return true;
  }
  return false;
}

}  // namespace geometry_base
}  // namespace maliput
