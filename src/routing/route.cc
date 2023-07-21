// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota. All rights reserved.
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
#include "maliput/routing/route.h"

#include <algorithm>
#include <array>
#include <iterator>

#include "maliput/geometry_base/strategy_base.h"

namespace maliput {
namespace routing {

Route::Route(const std::vector<Phase>& phases, const api::RoadNetwork* road_network)
    : phases_(phases), road_network_(road_network) {
  MALIPUT_THROW_UNLESS(!phases_.empty());
  MALIPUT_THROW_UNLESS(road_network_ != nullptr);
  /// TODO(#453): Validate end to end connection of the RoutePhases.
}

RoutePositionResult Route::FindRoutePosition(const api::InertialPosition& inertial_position) const {
  // Obtain the RoutePositionResult for each RoutePhase.
  std::vector<RoutePositionResult> route_position_results(phases_.size());
  for (int i = 0; i < static_cast<int>(phases_.size()); ++i) {
    route_position_results[i] = RoutePositionResult{i, phases_[i].FindPhasePosition(inertial_position)};
  }

  // Find the best RoutePositionResult.
  // Transform back to a RoadPositionResult and compare them.
  std::vector<api::RoadPositionResult> road_position_results;
  std::transform(
      route_position_results.begin(), route_position_results.end(), std::back_inserter(road_position_results),
      [road_geometry = road_network_->road_geometry(),
       &phases = std::as_const(phases_)](const auto& route_position_result) {
        const PhasePositionResult& phase_position_result = route_position_result.phase_position_result;
        const api::LaneSRange& lane_s_range =
            phases[route_position_result.phase_index].lane_s_ranges()[phase_position_result.lane_s_range_index];
        const api::Lane* lane = road_geometry->ById().GetLane(lane_s_range.lane_id());
        return api::RoadPositionResult{api::RoadPosition{lane, phase_position_result.lane_position},
                                       phase_position_result.inertial_position, phase_position_result.distance};
      });
  size_t best_result_index = 0;
  for (size_t i = 1; i < road_position_results.size(); ++i) {
    best_result_index =
        geometry_base::IsNewRoadPositionResultCloser(road_position_results[i], road_position_results[best_result_index])
            ? i
            : best_result_index;
  }

  return route_position_results[best_result_index];
}

RoutePositionResult Route::FindRoutePosition(const api::RoadPosition& road_position) const {
  MALIPUT_THROW_UNLESS(road_position.lane != nullptr);

  const auto route_phase_it = std::find_if(phases_.begin(), phases_.end(), [&road_position](const auto& phase) {
    return ValidatePositionIsInLaneSRanges(road_position, phase.lane_s_ranges(), phase.lane_s_range_tolerance());
  });

  // The provided road_position does not fall into any of the Phases within this Route.
  // Resort to finding the closest position using its Inertial position.
  if (route_phase_it == phases_.end()) {
    return FindRoutePosition(road_position.lane->ToInertialPosition(road_position.pos));
  }
  const int phase_index = static_cast<int>(std::distance(phases_.begin(), route_phase_it));
  return RoutePositionResult{phase_index, route_phase_it->FindPhasePosition(road_position)};
}

}  // namespace routing
}  // namespace maliput
