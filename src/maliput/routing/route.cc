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
#include <sstream>

#include "maliput/api/branch_point.h"
#include "maliput/api/compare.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/common/compare.h"
#include "maliput/geometry_base/strategy_base.h"

namespace maliput {
namespace routing {

Route::Route(const std::vector<Phase>& phases, const api::RoadNetwork* road_network)
    : phases_(phases), road_network_(road_network) {
  MALIPUT_THROW_UNLESS(!phases_.empty());
  MALIPUT_THROW_UNLESS(road_network_ != nullptr);

  // Populate the lane_id_to_indices_ dictionary.
  for (size_t phase_index = 0u; phase_index < phases_.size(); ++phase_index) {
    const std::vector<api::LaneSRange>& lane_s_ranges = phases_[phase_index].lane_s_ranges();
    for (size_t lane_s_range_index = 0u; lane_s_range_index < lane_s_ranges.size(); ++lane_s_range_index) {
      lane_id_to_indices_[lane_s_ranges[lane_s_range_index].lane_id()].push_back(
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
  const std::optional<LaneSRangeIndex> index_a = FindLaneSRangeIndex(lane_s_range_a);
  const std::optional<LaneSRangeIndex> index_b = FindLaneSRangeIndex(lane_s_range_b);

  // Determine whether lane_s_range_a and lane_s_range_b are in the same Route.
  if (!index_a.has_value() || !index_b.has_value()) {
    return LaneSRangeRelation::kUnknown;
  }

  // lane_s_range_b is unrelated to lane_s_range_a because they are in different non-adjacent phases.
  if (index_a->phase > index_b->phase + 1 || index_b->phase > index_a->phase + 1) {
    return LaneSRangeRelation::kUnrelated;
  }

  const size_t lane_s_range_b_next_index = index_b->lane_s_range + 1;
  const size_t lane_s_range_b_previous_index = index_b->lane_s_range - 1;

  // When both Phase indices are the same, we should look into the indices for the lane_s_ranges.
  if (index_a->phase == index_b->phase) {
    if (index_a->lane_s_range == index_b->lane_s_range) {
      return LaneSRangeRelation::kCoincident;
    } else if (index_a->lane_s_range == lane_s_range_b_next_index) {
      return LaneSRangeRelation::kAdjacentRight;
    } else if (index_a->lane_s_range > lane_s_range_b_next_index) {
      return LaneSRangeRelation::kRight;
    } else if (index_a->lane_s_range == lane_s_range_b_previous_index) {
      return LaneSRangeRelation::kAdjacentLeft;
    } else {  // (index_a->lane_s_range < lane_s_range_b_previous_index)
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
  if (index_a->phase == index_b->phase - 1) {
    const api::RoadPosition lane_s_range_a_road_pos =
        get_lane_s_range_road_position(index_a->phase, index_a->lane_s_range, kEnd);
    const api::RoadPosition lane_s_range_b_road_pos =
        get_lane_s_range_road_position(index_b->phase, index_b->lane_s_range, kStart);

    return kRelativePositionToSuceedingLaneSRange[static_cast<size_t>(
        ComputeRelativePosition(lane_s_range_a_road_pos, lane_s_range_b_road_pos, tolerance))];
  }

  // lane_s_range_b is behind of lane_s_range_a.
  // index_a->phase == index_b->phase - 1
  const api::RoadPosition lane_s_range_a_road_pos =
      get_lane_s_range_road_position(index_a->phase, index_a->lane_s_range, kStart);
  const api::RoadPosition lane_s_range_b_road_pos =
      get_lane_s_range_road_position(index_b->phase, index_b->lane_s_range, kEnd);

  return kRelativePositionToPreceedingLaneSRange[static_cast<size_t>(
      ComputeRelativePosition(lane_s_range_a_road_pos, lane_s_range_b_road_pos, tolerance))];
}

api::LaneSRoute Route::ComputeLaneSRoute(const api::RoadPosition& start_position) const {
  MALIPUT_THROW_UNLESS(start_position.lane != nullptr);

  const api::RoadPosition& end_position = end_route_position();
  // Creates a point like api::LaneSRange to find the appropriate LaneSRangeIndex of the start
  // and end positions.
  const api::LaneSRange end_point(end_position.lane->id(), api::SRange(end_position.pos.s(), end_position.pos.s()));
  const api::LaneSRange start_point(start_position.lane->id(),
                                    api::SRange(start_position.pos.s(), start_position.pos.s()));

  const std::optional<LaneSRangeIndex> start_index = FindLaneSRangeIndex(start_point);
  MALIPUT_THROW_UNLESS(start_index.has_value());

  const std::optional<LaneSRangeIndex> end_index = FindLaneSRangeIndex(end_point);
  MALIPUT_THROW_UNLESS(end_index.has_value());

  // Inserts @p index into the beginning of @p lane_s_ranges.
  auto prepend_to_route = [this](const LaneSRangeIndex& index, std::vector<api::LaneSRange>* lane_s_ranges) -> void {
    lane_s_ranges->insert(lane_s_ranges->begin(), GetLaneSRange(index.phase, index.lane_s_range));
  };

  // Returns true when @p index_a.phase is equal to @p index_b.phase.
  auto is_same_phase = [](const Route::LaneSRangeIndex& index_a, const Route::LaneSRangeIndex& index_b) -> bool {
    return index_a.phase == index_b.phase;
  };

  // The following double nested loop performs the routing within the Route from the ending api::LaneSRange
  // towards the starting api::LaneSRange. It is always a possible logical solution, and it is based on
  // one simple set of premises:
  //
  // 1- Move straight backwards first, otherwise
  // 2- Move towards the left / right, the one that offers the lesser number of hops with a possible
  //    straight backwards movement. Go to step 1.

  LaneSRangeIndex current_index = end_index.value();
  std::vector<api::LaneSRange> lane_s_ranges;

  // TODO(#543): this process should be simpler once we process in the constructor end-to-end Phase
  // connectivity as we can cache in the interfaces those LaneSRangeIndices that connect with each other.
  int num_phases_to_check = phases_.size() - start_index->phase;
  while (num_phases_to_check >= 0 && !is_same_phase(current_index, *start_index)) {
    num_phases_to_check--;
    // Adds the current LaneSRange to the last list.
    prepend_to_route(current_index, &lane_s_ranges);
    // Moves within the Phase towards the first LaneSRange with a straight predecessor.
    std::optional<LaneSRangeIndex> predecessor_index{};
    int num_lane_s_ranges_to_check = static_cast<int>(phases_[current_index.phase].lane_s_ranges().size());
    while (!predecessor_index.has_value() && num_lane_s_ranges_to_check >= 0) {
      predecessor_index = FindStraightPredecessor(current_index);
      if (!predecessor_index.has_value()) {
        current_index.lane_s_range += FindDirectionTowardsLaneSRangeWithStraightPredecessor(current_index);
        prepend_to_route(current_index, &lane_s_ranges);
      }
      num_lane_s_ranges_to_check--;
    }
    MALIPUT_VALIDATE(num_lane_s_ranges_to_check >= 0 && predecessor_index.has_value(),
                     "Failed to find an api::LaneSRange at Route::ComputeLaneSRoute that has a straight predecessor.");
    // Changes to the preceding Phase - LaneSRange.
    current_index = *predecessor_index;
  }
  // Check that we have found a solution.
  MALIPUT_VALIDATE(num_phases_to_check >= 0,
                   "Failed to find an api::LaneSRoute at Route::ComputeLaneSRoute. The backwards path finding exceeded "
                   "the number of phases to check.");

  // TODO(#543): make the set of LaneSRanges of the right length once we can easily map s-coordinates LaneSRange to
  // LaneSRange.
  // Prepend the remaining api::LaneSRanges within the initial Phase towards the starting api::LaneSRange.
  LaneSRangeRelation lane_s_range_relation{LaneSRangeRelation::kUnknown};
  while (lane_s_range_relation != LaneSRangeRelation::kCoincident) {
    prepend_to_route(current_index, &lane_s_ranges);
    const api::LaneSRange start_lane_s_range = GetLaneSRange(start_index->phase, start_index->lane_s_range);
    const api::LaneSRange current_lane_s_range = GetLaneSRange(current_index.phase, current_index.lane_s_range);
    lane_s_range_relation = ComputeLaneSRangeRelation(start_lane_s_range, current_lane_s_range);
    switch (lane_s_range_relation) {
      case LaneSRangeRelation::kLeft:
      case LaneSRangeRelation::kAdjacentLeft:
        current_index.lane_s_range += kTowardsRight;
        break;

      case LaneSRangeRelation::kRight:
      case LaneSRangeRelation::kAdjacentRight:
        current_index.lane_s_range += kTowardsLeft;
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
  if (lane_id_to_indices_.find(lane_s_range.lane_id()) == lane_id_to_indices_.end()) {
    return std::nullopt;
  }
  const double tolerance = road_network_->road_geometry()->linear_tolerance();
  const std::vector<LaneSRangeIndex> indices = lane_id_to_indices_.at(lane_s_range.lane_id());
  for (const LaneSRangeIndex& idx : indices) {
    if (phases_[idx.phase].lane_s_ranges()[idx.lane_s_range].Contains(lane_s_range, tolerance)) {
      return {idx};
    }
  }
  return std::nullopt;
}

std::optional<Route::LaneSRangeIndex> Route::FindStraightPredecessor(const Route::LaneSRangeIndex& index) const {
  MALIPUT_THROW_UNLESS(index.phase > 0u);
  const size_t preceeding_phase_index = index.phase - 1u;
  const Phase& preceeding_phase = phases_.at(preceeding_phase_index);
  const api::LaneSRange& lane_s_range = GetLaneSRange(index.phase, index.lane_s_range);
  const auto preceeding_lane_s_range_it =
      std::find_if(preceeding_phase.lane_s_ranges().begin(), preceeding_phase.lane_s_ranges().end(),
                   [this, lane_s_range](const api::LaneSRange& preceeding_lane_s_range) {
                     return ComputeLaneSRangeRelation(lane_s_range, preceeding_lane_s_range) ==
                            LaneSRangeRelation::kPreceedingStraight;
                   });
  if (preceeding_lane_s_range_it == preceeding_phase.lane_s_ranges().end()) {
    return std::nullopt;
  }
  const size_t preceeding_index =
      static_cast<size_t>(std::distance(preceeding_phase.lane_s_ranges().begin(), preceeding_lane_s_range_it));
  return {LaneSRangeIndex{preceeding_phase_index, preceeding_index}};
}

int Route::FindDirectionTowardsLaneSRangeWithStraightPredecessor(const Route::LaneSRangeIndex& index) const {
  const Phase& current_phase = Get(index.phase);
  const size_t min_current_index = 0u;
  const size_t max_current_index = current_phase.lane_s_ranges().size() - 1u;

  // Quick checks before looking for one side.
  if (index.lane_s_range == min_current_index) {  // This is the top right index, move to the left.
    return kTowardsLeft;
  } else if (index.lane_s_range == max_current_index) {  // This is the top right index, move to the right.
    return kTowardsRight;
  }

  // Evaluate if there are less elements to the right than to the left.
  if (index.lane_s_range <= (max_current_index - max_current_index)) {
    for (size_t i = index.lane_s_range - 1; i >= min_current_index; i--) {
      if (FindStraightPredecessor({index.phase, i} /* right_index */).has_value()) {
        return kTowardsRight;
      }
    }
    return kTowardsLeft;
  }
  for (size_t i = index.lane_s_range + 1; i <= max_current_index; i++) {
    if (FindStraightPredecessor({index.phase, i} /* left_index */).has_value()) {
      return kTowardsLeft;
    }
  }
  return kTowardsRight;
}

namespace {

// Finds a RoadPosition by matching @p lane pointer.
//
// @param lane Pointer to the api::Lane to match.
// @param positions Vector of positions to look for one with a matching api::Lane pointer with @p lane.
// @return An optional with a copy of the first position found with matching api::Lane pointer with @p lane.
std::optional<api::RoadPosition> FindPositionByLane(const api::Lane* lane,
                                                    const std::vector<api::RoadPosition>& positions) {
  const auto it = std::find_if(positions.begin(), positions.end(),
                               [lane](const api::RoadPosition& position) { return position.lane == lane; });
  return it != positions.end() ? std::optional<api::RoadPosition>{*it} : std::nullopt;
}

// Evaluates whether @p pos_a is close to @p pos_b with @p tolerance in the INERTIAL-Frame.
//
// @param pos_a Left hand side of the comparison.
// @param pos_b Right hand side of the comparison.
// @param tolerance Tolerance to compare the positions.
// @return true When @p pos_a and @p pos_b are within @p tolerance in the INERTIAL-Frame.
bool IsRoadPositionClose(const api::RoadPosition& pos_a, const api::RoadPosition& pos_b, double tolerance) {
  const api::InertialPosition& inertial_pos_a = pos_a.lane->ToInertialPosition(pos_a.pos);
  const api::InertialPosition& inertial_pos_b = pos_b.lane->ToInertialPosition(pos_b.pos);
  const common::ComparisonResult<api::InertialPosition> comparison_result =
      api::IsInertialPositionClose(inertial_pos_a, inertial_pos_b, tolerance);
  return !comparison_result.message.has_value();
}

// Returns the closest api::LaneEnd::Which of the api::Lane that @p position is in.
api::LaneEnd::Which GetClosestLaneEndWhich(const api::RoadPosition& position) {
  const api::Lane* lane = position.lane;
  return position.pos.s() <= lane->length() - position.pos.s() ? api::LaneEnd::Which::kStart
                                                               : api::LaneEnd::Which::kFinish;
}

// Matches @p position_a in the preceding Phase with an api::RoadPosition in @p positions_b from
// the succeeding Phase.
//
// Filters @p positions_b using @p ongoing_lane_end_set to look for valid positions in the topological
// sense to then find the first match in the subset by geometrical correspondence within @p tolerance.
//
// @param position_a api::RoadPosition at the end of the preceding Phase.
// @param positions_b api::RoadPositions at the start of the succeeding Phase.
// @param ongoing_lane_end_set The ongoing api::LaneEndSet that corresponds with @p position_a.
// @param tolerance Tolerance to compare the positions.
// @return An optional with an api::RoadPosition from @p positions_b that corresponds with @p position_a.
std::optional<api::RoadPosition> MatchPositionInLaneEndSet(const api::RoadPosition& position_a,
                                                           const std::vector<api::RoadPosition> positions_b,
                                                           const api::LaneEndSet& ongoing_lane_end_set,
                                                           double tolerance) {
  std::vector<api::RoadPosition> matching_positions_b{};
  std::copy_if(positions_b.begin(), positions_b.end(), std::back_inserter(matching_positions_b),
               [&ongoing_lane_end_set](const api::RoadPosition& position) {
                 for (int i = 0; i < ongoing_lane_end_set.size(); ++i) {
                   if (position.lane == ongoing_lane_end_set.get(i).lane) {
                     return true;
                   }
                 }
                 return false;
               });
  const auto it = std::find_if(matching_positions_b.begin(), matching_positions_b.end(),
                               [position_a, tolerance](const api::RoadPosition& position) {
                                 return IsRoadPositionClose(position_a, position, tolerance);
                               });
  return it != matching_positions_b.end() ? std::optional<api::RoadPosition>{*it} : std::nullopt;
}

}  // namespace

std::vector<std::string> Route::ValidateEndToEndConnectivity() const {
  std::vector<std::string> errors;

  // The start Phase must have only one start position.
  if (phases_.front().start_positions().size() != 1u) {
    errors.push_back("Start Phase::start_positions() does not have one position.");
  }
  // The end Phase must have only one end position.
  if (phases_.back().end_positions().size() != 1u) {
    errors.push_back("End Phase::end_positions() does not have one position.");
  }

  const double tolerance = road_network_->road_geometry()->linear_tolerance();

  for (size_t phase_index = 0u; phase_index < phases_.size() - 1; ++phase_index) {
    const Phase& phase_a = phases_[phase_index];
    const Phase& phase_b = phases_[phase_index + 1u];
    // The inter Phase interfaces must have the same count of points.
    if (phase_a.end_positions().size() != phase_b.start_positions().size()) {
      std::stringstream ss;
      ss << "Phases[" << phase_index << "].end_positions().size() is " << phase_a.end_positions().size();
      ss << "and Phases[" << (phase_index + 1u) << "].start_positions().size() is " << phase_b.start_positions().size();
      errors.push_back(ss.str());
    }

    // For each point in the end positions of phase_a, we need to find the correspondent
    // in the start positions of phase_b.
    // The topological connectivity logic follows:
    // - For each point in the end positions in phase_a, do
    //   - Try to find a point in the start positions of phase_b with the same api::LaneId. This occurs when the
    //     boundary between phase_a and phase_b happens in the middle of a lane.
    //     - If found, assert the points are coincident. Throw otherwise.
    //   - When not found, this means the boundary between phase_a and phase_b involves two different lanes. In this
    //     case, identify the api::BranchPoint and the respective ongoing set of LaneEnds.
    //     - Find which position of the end positions in phase_b belong to the ongoing set of LaneEnds.
    for (const api::RoadPosition& end_position : phase_a.end_positions()) {
      std::optional<api::RoadPosition> found_position_on_b =
          FindPositionByLane(end_position.lane, phase_b.start_positions());
      if (found_position_on_b) {
        if (!IsRoadPositionClose(end_position, *found_position_on_b, tolerance)) {
          std::stringstream ss;
          ss << "RoadPosition: {id: " << end_position.lane->id().string() << ", pos: {" << end_position.pos
             << "}} from Phases[" << phase_index
             << "] was found in the next Phase but position is not within tolerance.";
          errors.push_back(ss.str());
        }
      } else {
        const api::LaneEndSet* lane_end_set =
            end_position.lane->GetOngoingBranches(GetClosestLaneEndWhich(end_position));
        found_position_on_b =
            MatchPositionInLaneEndSet(end_position, phase_b.start_positions(), *lane_end_set, tolerance);
        if (!found_position_on_b) {
          std::stringstream ss;
          ss << "RoadPosition: {id: " << end_position.lane->id().string() << ", pos: {" << end_position.pos
             << "}} from Phases[" << phase_index << "] was not found in next Phase within tolerance.";
          errors.push_back(ss.str());
        }
      }
    }
  }
  return errors;
}

}  // namespace routing
}  // namespace maliput
