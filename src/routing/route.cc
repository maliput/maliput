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

namespace {

enum class RelativePosition { kLeft = 0, kCenter, kRight };

RelativePosition ComputeRelativePositionFor(const api::RoadPosition& pos_a, const api::RoadPosition& pos_b,
                                            double tolerance) {
  const api::InertialPosition inerital_pos_a = pos_a.lane->ToInertialPosition(pos_a.pos);
  const api::InertialPosition inerital_pos_b = pos_b.lane->ToInertialPosition(pos_b.pos);

  // When points are within tolerance, they are considered the same.
  const math::Vector3 b_to_a = inerital_pos_b.xyz() - inerital_pos_a.xyz();
  if (b_to_a.norm() <= tolerance) {
    return RelativePosition::kCenter;
  }

  const api::Rotation inertial_rotation_a = pos_a.lane->GetOrientation(pos_a.pos);
  const math::Vector3 s_hat_a = inertial_rotation_a.Apply({1., 0., 0.}).xyz();
  const math::Vector3 h_hat_a = inertial_rotation_a.Apply({0., 0., 1.}).xyz();

  const math::Vector3 norm_b_to_a = (inerital_pos_b.xyz() - inerital_pos_a.xyz()).normalized();

  const bool is_to_left = s_hat_a.cross(norm_b_to_a).dot(h_hat_a) > 0.;
  return is_to_left ? RelativePosition::kLeft : RelativePosition::kRight;
}

}  // namespace

LaneSRangeRelation Route::ComputeLaneSRangeRelation(const api::LaneSRange& lane_s_range_a,
                                                    const api::LaneSRange& lane_s_range_b) const {
  // Find index of lane_s_range_a and lane_s_range_b.
  const std::optional<LaneSRangeIndex> lane_s_range_a_index = FindLaneSRangeIndex(lane_s_range_a);
  const std::optional<LaneSRangeIndex> lane_s_range_b_index = FindLaneSRangeIndex(lane_s_range_b);

  // Determine whether lane_s_range_a and lane_s_range_b are in the same Route.
  if (!lane_s_range_a_index.has_value() || !lane_s_range_b_index.has_value()) {
    return LaneSRangeRelation::kUnknown;
  }

  // lane_s_range_b is unrelated to lane_s_range_a since they are in non-adjacent phases.
  if (lane_s_range_a_index->first > lane_s_range_b_index->first + 1 ||
      lane_s_range_b_index->first > lane_s_range_a_index->first + 1) {
    return LaneSRangeRelation::kUnrelated;
  }

  const size_t lane_s_range_b_next_index = lane_s_range_b_index->second + 1;
  const size_t lane_s_range_b_previous_index = lane_s_range_b_index->second - 1;

  // When both phase indices are the same, we should look into the indices for the lane_s_ranges.
  if (lane_s_range_a_index->first == lane_s_range_b_index->first) {
    if (lane_s_range_a_index->second == lane_s_range_b_index->second) {
      return LaneSRangeRelation::kCoincident;
    } else if (lane_s_range_a_index->second == lane_s_range_b_next_index) {
      return LaneSRangeRelation::kAdjacentRight;
    } else if (lane_s_range_a_index->second > lane_s_range_b_next_index) {
      return LaneSRangeRelation::kRight;
    } else if (lane_s_range_a_index->second == lane_s_range_b_previous_index) {
      return LaneSRangeRelation::kAdjacentLeft;
    } else {  // (lane_s_range_a_index->second < lane_s_range_b_previous_index)
      return LaneSRangeRelation::kLeft;
    }
  }

  auto get_lane_s_range_road_position = [&](size_t route_phase_index, size_t lane_range_index, bool start) {
    const auto& lane_s_range = phases_[route_phase_index].lane_s_ranges()[lane_range_index];
    const api::Lane* lane = road_network_->road_geometry()->ById().GetLane(lane_s_range.lane_id());
    return api::RoadPosition(
        lane, api::LanePosition(start ? lane_s_range.s_range().s0() : lane_s_range.s_range().s1(), 0., 0.));
  };

  static constexpr bool kStart{true};
  static constexpr bool kEnd{!kStart};
  static constexpr std::array<LaneSRangeRelation, 3> kRelativePositionToSuceedingLaneSRange{
      LaneSRangeRelation::kSucceedingLeft, LaneSRangeRelation::kSucceedingStraight,
      LaneSRangeRelation::kSucceedingRight};
  static constexpr std::array<LaneSRangeRelation, 3> kRelativePositionToPreceedingLaneSRange{
      LaneSRangeRelation::kPreceedingLeft, LaneSRangeRelation::kPreceedingStraight,
      LaneSRangeRelation::kPreceedingRight};
  const double tolerance = road_network_->road_geometry()->linear_tolerance();

  // Determine whether lane_s_range_b is ahead of lane_s_range_a.
  if (lane_s_range_a_index->first == lane_s_range_b_index->first - 1) {
    const api::RoadPosition lane_s_range_a_road_pos =
        get_lane_s_range_road_position(lane_s_range_a_index->first, lane_s_range_a_index->second, kEnd);
    const api::RoadPosition lane_s_range_b_road_pos =
        get_lane_s_range_road_position(lane_s_range_b_index->first, lane_s_range_b_index->second, kStart);

    return kRelativePositionToSuceedingLaneSRange[static_cast<size_t>(
        ComputeRelativePositionFor(lane_s_range_a_road_pos, lane_s_range_b_road_pos, tolerance))];
  }

  // lane_s_range_b is behind of lane_s_range_a.
  // lane_s_range_a_index->first == lane_s_range_b_index->first - 1
  const api::RoadPosition lane_s_range_a_road_pos =
      get_lane_s_range_road_position(lane_s_range_a_index->first, lane_s_range_a_index->second, kStart);
  const api::RoadPosition lane_s_range_b_road_pos =
      get_lane_s_range_road_position(lane_s_range_b_index->first, lane_s_range_b_index->second, kEnd);

  return kRelativePositionToPreceedingLaneSRange[static_cast<size_t>(
      ComputeRelativePositionFor(lane_s_range_a_road_pos, lane_s_range_b_road_pos, tolerance))];
}

std::optional<Route::LaneSRangeIndex> Route::FindLaneSRangeIndex(const api::LaneSRange& lane_s_range) const {
  auto is_lane_s_range_contained = [tolerance = road_network_->road_geometry()->linear_tolerance()](
                                       const api::LaneSRange& lane_s_range_a, const api::LaneSRange& lane_s_range_b) {
    if (lane_s_range_a.lane_id() != lane_s_range_b.lane_id()) {
      return false;
    }
    const bool s1_is_in_range = lane_s_range_a.s_range().s1() + tolerance >= lane_s_range_b.s_range().s1() &&
                                lane_s_range_a.s_range().s0() - tolerance <= lane_s_range_b.s_range().s1();
    const bool s0_is_in_range = lane_s_range_a.s_range().s1() + tolerance >= lane_s_range_b.s_range().s0() &&
                                lane_s_range_a.s_range().s0() - tolerance <= lane_s_range_b.s_range().s0();
    return s1_is_in_range && s0_is_in_range;
  };

  for (size_t i = 0; i < phases_.size(); ++i) {
    for (size_t j = 0; j < phases_[i].lane_s_ranges().size(); ++j) {
      if (is_lane_s_range_contained(phases_[i].lane_s_ranges()[j], lane_s_range)) {
        return {std::make_pair(i, j)};
      }
    }
  }
  return {};
}

}  // namespace routing
}  // namespace maliput
