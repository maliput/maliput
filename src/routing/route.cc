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
  /// TODO(#453): Validate end to end connection of the Phases.

  // Populate the lane_id_to_lane_s_range_indices_ dictionary.
  for (int phase_index = 0; phase_index < static_cast<int>(phases_.size()); ++phase_index) {
    const std::vector<api::LaneSRange>& lane_s_ranges = phases_[phase_index].lane_s_ranges();
    for (int lane_s_range_index = 0; lane_s_range_index < static_cast<int>(lane_s_ranges.size());
         ++lane_s_range_index) {
      lane_id_to_lane_s_range_indices_[lane_s_ranges[lane_s_range_index].lane_id()].push_back(
          LaneSRangeIndex{phase_index, lane_s_range_index});
    }
  }
}

RoutePositionResult Route::FindRoutePosition(const api::InertialPosition& inertial_position) const {
  // Obtain the RoutePositionResult for each Phase.
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

  const auto phase_it = std::find_if(phases_.begin(), phases_.end(), [&road_position](const auto& phase) {
    return ValidatePositionIsInLaneSRanges(road_position, phase.lane_s_ranges(), phase.lane_s_range_tolerance());
  });

  // The provided road_position does not fall into any of the Phases within this Route.
  // Resort to finding the closest position using its Inertial position.
  if (phase_it == phases_.end()) {
    return FindRoutePosition(road_position.lane->ToInertialPosition(road_position.pos));
  }
  const int phase_index = static_cast<int>(std::distance(phases_.begin(), phase_it));
  return RoutePositionResult{phase_index, phase_it->FindPhasePosition(road_position)};
}

namespace {

enum class RelativePosition { kLeft = 0, kCenter, kRight };

RelativePosition ComputeRelativePosition(const api::RoadPosition& pos_a, const api::RoadPosition& pos_b,
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

  // lane_s_range_b is unrelated to lane_s_range_a
  if (lane_s_range_a_index->first > lane_s_range_b_index->first + 1 ||
      lane_s_range_b_index->first > lane_s_range_a_index->first + 1) {
    return LaneSRangeRelation::kUnrelated;
  }

  const size_t lane_s_range_b_next_index = lane_s_range_b_index->second + 1;
  const size_t lane_s_range_b_previous_index = lane_s_range_b_index->second - 1;

  // When both indices are the same, we should look into the indices for the lane_s_ranges.
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

  auto get_lane_s_range_road_position = [&](size_t phase_index, size_t lane_range_index, bool start) {
    const auto& lane_s_range = phases_[phase_index].lane_s_ranges()[lane_range_index];
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
        ComputeRelativePosition(lane_s_range_a_road_pos, lane_s_range_b_road_pos, tolerance))];
  }

  // lane_s_range_b is behind of lane_s_range_a.
  // lane_s_range_a_index->first == lane_s_range_b_index->first - 1
  const api::RoadPosition lane_s_range_a_road_pos =
      get_lane_s_range_road_position(lane_s_range_a_index->first, lane_s_range_a_index->second, kStart);
  const api::RoadPosition lane_s_range_b_road_pos =
      get_lane_s_range_road_position(lane_s_range_b_index->first, lane_s_range_b_index->second, kEnd);

  return kRelativePositionToPreceedingLaneSRange[static_cast<size_t>(
      ComputeRelativePosition(lane_s_range_a_road_pos, lane_s_range_b_road_pos, tolerance))];
}

api::LaneSRoute Route::ComputeLaneSRoute(const api::RoadPosition& start_position) const {
  MALIPUT_THROW_UNLESS(start_position.lane != nullptr);

  const api::RoadPosition& end_road_position = end_route_position();
  // Creates a point like api::LaneSRange to find the appropriate LaneSRangeIndex of the start
  // and end positions.
  const api::LaneSRange end_lane_s_range_point(end_road_position.lane->id(),
                                               api::SRange(end_road_position.pos.s(), end_road_position.pos.s()));
  const api::LaneSRange start_lane_s_range_point(start_position.lane->id(),
                                                 api::SRange(start_position.pos.s(), start_position.pos.s()));

  const std::optional<LaneSRangeIndex> start_lane_s_range_index = FindLaneSRangeIndex(start_lane_s_range_point);
  MALIPUT_THROW_UNLESS(start_lane_s_range_index.has_value());

  const std::optional<LaneSRangeIndex> end_lane_s_range_index = FindLaneSRangeIndex(end_lane_s_range_point);
  MALIPUT_THROW_UNLESS(end_lane_s_range_index.has_value());

  // Inserts @p lane_s_range_index into @p lane_s_range at the beginning.
  auto prepend_lane_s_range_to_route = [this](const LaneSRangeIndex& lane_s_range_index,
                                              std::vector<api::LaneSRange>* lane_s_ranges) -> void {
    lane_s_ranges->insert(lane_s_ranges->begin(), GetLaneSRange(lane_s_range_index.first, lane_s_range_index.second));
  };

  // Returns true when @p lane_s_range_index_a.first is equal to @p lane_s_range_index_b.first.
  auto is_same_phase = [](const Route::LaneSRangeIndex& lane_s_range_index_a,
                          const Route::LaneSRangeIndex& lane_s_range_index_b) -> bool {
    return lane_s_range_index_a.first == lane_s_range_index_b.first;
  };

  LaneSRangeIndex current_lane_s_range_index = end_lane_s_range_index.value();
  std::vector<api::LaneSRange> lane_s_ranges;

  // The following double nested loop performs the routing within the Route from the ending api::LaneSRange
  // towards the starting api::LaneSRange. The is always a possible logical solution, and it is based on
  // one simple set of premises:
  //
  // 1- Move straight backwards first, otherwise
  // 2- Move towards the left / right, the one that offers the lesser number of hops with a possible
  //    straight backwards movement. Go to step 1.

  // TODO(#543): this process should be simpler once we process in the constructor end-to-end Phase
  // connectivity as we can cache in the interfaces those LaneSRangeIndices that connect with each other.
  while (!is_same_phase(current_lane_s_range_index, *start_lane_s_range_index)) {
    // Adds the current LaneSRange to the last list.
    prepend_lane_s_range_to_route(current_lane_s_range_index, &lane_s_ranges);
    // Moves within the Phase towards the first LaneSRange with a straight predecessor.
    std::optional<LaneSRangeIndex> predecessor_lane_s_range_index{};
    while (!predecessor_lane_s_range_index.has_value()) {
      predecessor_lane_s_range_index = FindStraightPredecessor(current_lane_s_range_index);
      if (!predecessor_lane_s_range_index.has_value()) {
        current_lane_s_range_index.second +=
            FindDirectionTowardsLaneSRangeWithStraightPredecessor(current_lane_s_range_index);
        prepend_lane_s_range_to_route(current_lane_s_range_index, &lane_s_ranges);
      }
    }
    // Changes to the preceeding Phase - LaneSRange.
    current_lane_s_range_index = *predecessor_lane_s_range_index;
  }

  // TODO(#543): make the set of LaneSRanges of the right length once we can easily map s-coordinates LaneSRange to
  // LaneSRange.
  LaneSRangeRelation lane_s_range_relation{LaneSRangeRelation::kUnknown};
  while (lane_s_range_relation != LaneSRangeRelation::kCoincident) {
    prepend_lane_s_range_to_route(current_lane_s_range_index, &lane_s_ranges);
    const api::LaneSRange start_lane_s_range =
        GetLaneSRange(start_lane_s_range_index->first, start_lane_s_range_index->second);
    const api::LaneSRange current_lane_s_range =
        GetLaneSRange(current_lane_s_range_index.first, current_lane_s_range_index.second);
    lane_s_range_relation = ComputeLaneSRangeRelation(start_lane_s_range, current_lane_s_range);
    switch (lane_s_range_relation) {
      case LaneSRangeRelation::kLeft:
      case LaneSRangeRelation::kAdjacentLeft:
        current_lane_s_range_index.second += kTowardsRight;
        break;

      case LaneSRangeRelation::kRight:
      case LaneSRangeRelation::kAdjacentRight:
        current_lane_s_range_index.second += kTowardsLeft;
        break;

      case LaneSRangeRelation::kCoincident:
        // Do nothing, leaving the loop now.
        break;

      default:
        MALIPUT_THROW_MESSAGE("Code should not arrive here.");
        break;
    };
  }
  return api::LaneSRoute(lane_s_ranges);
}

std::optional<Route::LaneSRangeIndex> Route::FindLaneSRangeIndex(const api::LaneSRange& lane_s_range) const {
  if (lane_id_to_lane_s_range_indices_.find(lane_s_range.lane_id()) == lane_id_to_lane_s_range_indices_.end()) {
    return {};
  }
  const double tolerance = road_network_->road_geometry()->linear_tolerance();
  const std::vector<LaneSRangeIndex> lane_s_range_indices = lane_id_to_lane_s_range_indices_.at(lane_s_range.lane_id());
  for (const LaneSRangeIndex& idx : lane_s_range_indices) {
    if (phases_[idx.first].lane_s_ranges()[idx.second].Contains(lane_s_range, tolerance)) {
      return {idx};
    }
  }
  return {};
}

std::optional<Route::LaneSRangeIndex> Route::FindStraightPredecessor(
    const Route::LaneSRangeIndex& lane_s_range_index) const {
  const size_t preceeding_phase_index = lane_s_range_index.first - 1u;
  const Phase& preceeding_phase = phases_.at(preceeding_phase_index);
  const api::LaneSRange& lane_s_range = GetLaneSRange(lane_s_range_index.first, lane_s_range_index.second);
  const auto preceeding_lane_s_range_it =
      std::find_if(preceeding_phase.lane_s_ranges().begin(), preceeding_phase.lane_s_ranges().end(),
                   [this, lane_s_range](const api::LaneSRange& preceeding_lane_s_range) {
                     return ComputeLaneSRangeRelation(lane_s_range, preceeding_lane_s_range) ==
                            LaneSRangeRelation::kPreceedingStraight;
                   });
  if (preceeding_lane_s_range_it == preceeding_phase.lane_s_ranges().end()) {
    return {};
  }
  const size_t preceeding_lane_s_range_index =
      static_cast<size_t>(std::distance(preceeding_phase.lane_s_ranges().begin(), preceeding_lane_s_range_it));
  return {LaneSRangeIndex{preceeding_phase_index, preceeding_lane_s_range_index}};
}

int Route::FindDirectionTowardsLaneSRangeWithStraightPredecessor(
    const Route::LaneSRangeIndex& lane_s_range_index) const {
  const Phase& current_phase = Get(lane_s_range_index.first);
  const size_t min_current_lane_s_range_index = 0u;
  const size_t max_current_lane_s_range_index = current_phase.lane_s_ranges().size() - 1u;

  // Quick checks before looking for one side.
  if (lane_s_range_index.second == min_current_lane_s_range_index) {  // This is the top right index, move to the left.
    return kTowardsLeft;
  } else if (lane_s_range_index.second ==
             max_current_lane_s_range_index) {  // This is the top right index, move to the left.
    return kTowardsRight;
  }

  // Evaluate if there are less elements to the right than to the left.
  if (lane_s_range_index.second <= (max_current_lane_s_range_index - max_current_lane_s_range_index)) {
    for (size_t i = lane_s_range_index.second - 1; i >= min_current_lane_s_range_index; i--) {
      if (FindStraightPredecessor({lane_s_range_index.first, i} /* right_lane_s_range_index */).has_value()) {
        return kTowardsRight;
      }
    }
    return kTowardsLeft;
  }
  for (size_t i = lane_s_range_index.second + 1; i <= max_current_lane_s_range_index; i++) {
    if (FindStraightPredecessor({lane_s_range_index.first, i} /* left_lane_s_range_index */).has_value()) {
      return kTowardsLeft;
    }
  }
  return kTowardsRight;
}

}  // namespace routing
}  // namespace maliput
