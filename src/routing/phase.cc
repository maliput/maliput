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
#include "maliput/routing/phase.h"

#include <algorithm>
#include <iterator>

#include "maliput/api/lane.h"
#include "maliput/api/road_geometry.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/geometry_base/strategy_base.h"

namespace maliput {
namespace routing {
namespace {

// @return true When @p value is within @p min and @p max, inclusive.
inline bool is_in_range(double min, double max, double value) { return value >= min && value <= max; }

// @return true When @p lane_s_range is a valid range in @p road_geometry with @p tolerance in the LANE-Frame s
// coordinate.
bool ValidateLaneSRangeIsInRoadNetwork(const api::LaneSRange& lane_s_range, const api::RoadGeometry* road_geometry,
                                       double tolerance) {
  const api::Lane* lane = road_geometry->ById().GetLane(lane_s_range.lane_id());
  if (lane == nullptr) {
    return false;
  }
  const double min_s_range = -tolerance;
  const double max_s_range = lane->length() + tolerance;
  return is_in_range(min_s_range, max_s_range, lane_s_range.s_range().s0()) &&
         is_in_range(min_s_range, max_s_range, lane_s_range.s_range().s1());
}

// @return true When @p lane_s_range_a and @p lane_s_range_b are adjacent.
bool ValidateLaneSRangesAreAdjancent(const api::LaneSRange& lane_s_range_a, const api::LaneSRange& lane_s_range_b,
                                     const api::RoadGeometry* road_geometry) {
  const api::Lane* lane_a = road_geometry->ById().GetLane(lane_s_range_a.lane_id());
  const api::Lane* lane_b = road_geometry->ById().GetLane(lane_s_range_b.lane_id());
  return (lane_a->to_left() == lane_b && lane_b->to_right() == lane_a) ||
         (lane_b->to_left() == lane_a && lane_a->to_right() == lane_b);
}

}  // namespace

bool ValidatePositionIsInLaneSRanges(const maliput::api::RoadPosition& position,
                                     const std::vector<api::LaneSRange>& lane_s_ranges, double tolerance) {
  MALIPUT_THROW_UNLESS(position.lane != nullptr);
  MALIPUT_THROW_UNLESS(!lane_s_ranges.empty());
  MALIPUT_THROW_UNLESS(tolerance >= 0.);

  return std::any_of(lane_s_ranges.begin(), lane_s_ranges.end(), [position, tolerance](const auto& lane_s_range) {
    if (position.lane->id() != lane_s_range.lane_id()) {
      return false;
    }
    const double min_s_range =
        (lane_s_range.s_range().WithS() ? lane_s_range.s_range().s0() : lane_s_range.s_range().s1()) - tolerance;
    const double max_s_range =
        (lane_s_range.s_range().WithS() ? lane_s_range.s_range().s1() : lane_s_range.s_range().s0()) + tolerance;
    return is_in_range(min_s_range, max_s_range, position.pos.s());
  });
}

Phase::Phase(int index, double lane_s_range_tolerance, const std::vector<api::RoadPosition>& start_positions,
             const std::vector<api::RoadPosition>& end_positions, const std::vector<api::LaneSRange>& lane_s_ranges,
             const api::RoadNetwork* road_network)
    : index_(index),
      lane_s_range_tolerance_(lane_s_range_tolerance),
      start_positions_(start_positions),
      end_positions_(end_positions),
      lane_s_ranges_(lane_s_ranges),
      road_network_(road_network) {
  MALIPUT_THROW_UNLESS(index_ >= 0);
  MALIPUT_THROW_UNLESS(lane_s_range_tolerance >= 0.);
  MALIPUT_THROW_UNLESS(!start_positions_.empty());
  MALIPUT_THROW_UNLESS(std::all_of(start_positions_.begin(), start_positions_.end(),
                                   [](const auto& pos) { return pos.lane != nullptr; }));
  MALIPUT_THROW_UNLESS(!end_positions_.empty());
  MALIPUT_THROW_UNLESS(
      std::all_of(end_positions_.begin(), end_positions_.end(), [](const auto& pos) { return pos.lane != nullptr; }));
  MALIPUT_THROW_UNLESS(!lane_s_ranges_.empty());
  MALIPUT_THROW_UNLESS(road_network_ != nullptr);
  const api::RoadGeometry* road_geometry = road_network_->road_geometry();
  MALIPUT_VALIDATE(std::all_of(lane_s_ranges_.begin(), lane_s_ranges_.end(),
                               [&](const auto& lane_s_range) {
                                 return ValidateLaneSRangeIsInRoadNetwork(lane_s_range, road_geometry,
                                                                          lane_s_range_tolerance_);
                               }),
                   "LaneSRange is not in the road_network_.");
  MALIPUT_VALIDATE(std::all_of(start_positions_.begin(), start_positions_.end(),
                               [&](const auto& position) {
                                 return ValidatePositionIsInLaneSRanges(position, lane_s_ranges_,
                                                                        lane_s_range_tolerance_);
                               }),
                   "Start position is not in lane_s_ranges_.");
  MALIPUT_VALIDATE(std::all_of(end_positions_.begin(), end_positions_.end(),
                               [&](const auto& position) {
                                 return ValidatePositionIsInLaneSRanges(position, lane_s_ranges_,
                                                                        lane_s_range_tolerance_);
                               }),
                   "End position is not in lane_s_ranges_.");
  for (size_t i = 0; i < lane_s_ranges_.size() - 1u; ++i) {
    MALIPUT_VALIDATE(ValidateLaneSRangesAreAdjancent(lane_s_ranges_[i], lane_s_ranges_[i + 1], road_geometry),
                     "LaneSRanges are not adjacent.");
  }
}

PhasePositionResult Phase::FindPhasePosition(const api::InertialPosition& inertial_position) const {
  std::vector<api::RoadPositionResult> road_position_results;
  const api::RoadGeometry* road_geometry = road_network_->road_geometry();
  std::transform(
      lane_s_ranges_.cbegin(), lane_s_ranges_.cend(), std::back_inserter(road_position_results),
      [road_geometry, &inertial_position, &tolerance = lane_s_range_tolerance_](const auto& lane_s_range) {
        const api::Lane* lane = road_geometry->ById().GetLane(lane_s_range.lane_id());
        const api::LanePositionResult lane_position_result = lane->ToLanePosition(inertial_position);
        api::RoadPosition road_position{lane, lane_position_result.lane_position};
        if (ValidatePositionIsInLaneSRanges(road_position, {lane_s_range}, tolerance)) {
          return api::RoadPositionResult{road_position, lane_position_result.nearest_position,
                                         lane_position_result.distance};
        }
        // Find the best candidate within the lane_s_range extent. Criteria follows:
        // - Match the extent (s0 or s1) that is closer to the lane returned lane_position_result.
        // - Create a new api::LanePosition in the center of the api::Lane whose s-coordinate
        //   is equal to the closer extent (s0 or s1).
        // - Complete the api::RoadPositionResults with that new api::LanePosition.
        const double distance_to_s0 = std::abs(lane_position_result.lane_position.s() - lane_s_range.s_range().s0());
        const double distance_to_s1 = std::abs(lane_position_result.lane_position.s() - lane_s_range.s_range().s1());
        const api::LanePosition clamped_lane_position{
            distance_to_s0 <= distance_to_s1 ? lane_s_range.s_range().s0() : lane_s_range.s_range().s1(), 0., 0.};
        const api::InertialPosition nearest_position = lane->ToInertialPosition(clamped_lane_position);
        const double distance = nearest_position.Distance(inertial_position);
        return api::RoadPositionResult{api::RoadPosition{lane, clamped_lane_position}, nearest_position, distance};
      });

  size_t best_candidate_index = 0u;
  for (size_t i = 1u; i < lane_s_ranges_.size(); ++i) {
    best_candidate_index = geometry_base::IsNewRoadPositionResultCloser(road_position_results[i],
                                                                        road_position_results[best_candidate_index])
                               ? i
                               : best_candidate_index;
  }

  return PhasePositionResult{static_cast<int>(best_candidate_index),
                             road_position_results[best_candidate_index].road_position.pos,
                             road_position_results[best_candidate_index].nearest_position,
                             road_position_results[best_candidate_index].distance};
}

PhasePositionResult Phase::FindPhasePosition(const api::RoadPosition& road_position) const {
  MALIPUT_THROW_UNLESS(road_position.lane != nullptr);

  if (!ValidatePositionIsInLaneSRanges(road_position, lane_s_ranges_, lane_s_range_tolerance_)) {
    return FindPhasePosition(road_position.lane->ToInertialPosition(road_position.pos));
  }
  const auto best_candidate_it = std::find_if(
      lane_s_ranges_.begin(), lane_s_ranges_.end(),
      [lane_id = road_position.lane->id()](const auto& lane_s_range) { return lane_s_range.lane_id() == lane_id; });
  const size_t best_candidate_index = std::distance(lane_s_ranges_.begin(), best_candidate_it);
  const api::InertialPosition inertial_position =
      road_position.lane->ToInertialPosition({road_position.pos.s(), 0., 0.});
  const api::LanePositionResult lane_position_result = road_position.lane->ToLanePosition(inertial_position);
  return PhasePositionResult{static_cast<int>(best_candidate_index), lane_position_result.lane_position,
                             lane_position_result.nearest_position, lane_position_result.distance};
}

}  // namespace routing
}  // namespace maliput
