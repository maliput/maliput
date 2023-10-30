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

#include "maliput/geometry_base/brute_force_strategy.h"

#include <limits>

#include <maliput/geometry_base/filter_positions.h>

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace geometry_base {

maliput::api::RoadPositionResult BruteForceStrategy::DoToRoadPosition(
    const maliput::api::InertialPosition& inertial_pos, const std::optional<maliput::api::RoadPosition>& hint) const {
  maliput::api::RoadPositionResult result;
  if (hint.has_value()) {
    MALIPUT_THROW_UNLESS(hint->lane != nullptr);
    const maliput::api::LanePositionResult lane_pos = hint->lane->ToLanePosition(inertial_pos);
    result = maliput::api::RoadPositionResult{
        {hint->lane, lane_pos.lane_position}, lane_pos.nearest_position, lane_pos.distance};
  } else {
    const std::vector<maliput::api::RoadPositionResult> road_position_results =
        DoFindRoadPositions(inertial_pos, std::numeric_limits<double>::infinity());
    MALIPUT_THROW_UNLESS(road_position_results.size());

    // Filter the candidates within a linear tolerance of distance.
    const std::vector<maliput::api::RoadPositionResult> near_road_positions_results =
        maliput::geometry_base::FilterRoadPositionResults(
            road_position_results,
            [tol = get_road_geometry()->linear_tolerance()](const maliput::api::RoadPositionResult& result) {
              return result.distance <= tol;
            });

    // If it is empty then I should use all the road position results.
    const std::vector<maliput::api::RoadPositionResult>& filtered_road_position_results =
        near_road_positions_results.empty() ? road_position_results : near_road_positions_results;
    result = filtered_road_position_results[0];
    for (const auto& road_position_result : filtered_road_position_results) {
      if (IsNewRoadPositionResultCloser(road_position_result, result)) {
        result = road_position_result;
      }
    }
  }
  return result;
}

std::vector<maliput::api::RoadPositionResult> BruteForceStrategy::DoFindRoadPositions(
    const maliput::api::InertialPosition& inertial_position, double radius) const {
  const api::RoadGeometry* rg = get_road_geometry();
  MALIPUT_THROW_UNLESS(rg != nullptr);
  MALIPUT_THROW_UNLESS(radius >= 0.);

  std::vector<maliput::api::RoadPositionResult> road_position_results;

  for (int i = 0; i < rg->num_junctions(); ++i) {
    const maliput::api::Junction* junction = rg->junction(i);
    MALIPUT_THROW_UNLESS(junction != nullptr);
    for (int j = 0; j < junction->num_segments(); ++j) {
      const maliput::api::Segment* segment = junction->segment(j);
      MALIPUT_THROW_UNLESS(segment != nullptr);
      for (int k = 0; k < segment->num_lanes(); ++k) {
        const api::Lane* lane = segment->lane(k);
        MALIPUT_THROW_UNLESS(lane != nullptr);
        maliput::api::InertialPosition nearest_position;
        const maliput::api::LanePositionResult result = lane->ToLanePosition(inertial_position);
        if (radius == std::numeric_limits<double>::infinity() || result.distance <= radius) {
          road_position_results.push_back(
              {api::RoadPosition(lane, result.lane_position), result.nearest_position, result.distance});
        }
      }
    }
  }

  return road_position_results;
}

}  // namespace geometry_base
}  // namespace maliput
