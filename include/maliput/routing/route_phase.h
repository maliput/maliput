// BSD 3-Clause License
//
// Copyright (c) 2023, Woven Planet. All rights reserved.
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

#include <optional>
#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_network.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace routing {

/// Manages a phase in a complete Route, towards its completion of a Route.
///
/// It is composed of a default maliput::api::LaneSRange which is the corridor
/// the agent traversing the Route should follow. Alternative parallel adjacent
/// LaneSRanges might be taken when provided.
///
/// (To be discussed): we could split the vector of adjacent LaneSRanges into
/// two dictionaries (adjacent to the left and adjacent to the right).
///
/// When the start and end maliput::api::RoadPositions belong to different
/// maliput::api::Lanes, a switch of maliput::api::Lanes is required.
///
/// (To be discussed): we can attach the start and / or end road_positions to
/// the default_lane_s_range as a precondition.
///
/// (To be discussed) As a proxy for agents, the RoadNetwork pointer is provided
/// to execute convenient queries for the agent, such as retrieving the rules
/// that apply to specific corridors of this RoutePhase.
///
/// Agents can localize themselves within a RoutePhase by using FindLaneSRangeBy()
/// methods. This is useful when they are initially placing themselves on a path
/// or for iterative querying.
class RoutePhase final {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoutePhase);
  RoutePhase() = delete;

  /// Constructs a RoutePhase.
  ///
  /// @param index The index at the parent Route. It must not be negative.
  /// @param start_road_position The start maliput::api::RoadPosition of this
  /// RoutePhase. `start_road_position.lane` must not be nullptr and it must be
  /// in either the @p default_lane_s_range or @p adjacent_lane_s_ranges.
  /// @param end_road_position The end maliput::api::RoadPosition of this
  /// RoutePhase. `end_road_position.lane` must not be nullptr and it must be
  /// in either the @p default_lane_s_range or @p adjacent_lane_s_ranges.
  /// @param lane_s_range_tolerance Tolerance to compare maliput::api::LaneSRanges.
  /// It must not be negative.
  /// @param default_lane_s_range The default maliput::api::LaneSRanges of this phase.
  /// @param adjancent_lane_s_ranges List of adjacent maliput::api::LaneSRanges.
  /// @param road_network The pointer to the maliput::api::RoadNetwork. It must
  /// not be nullptr.
  /// @throws maliput::common::assertion_error When @p index is negative.
  /// @throws maliput::common::assertion_error When @p lane_s_range_tolerance is
  /// negative.
  /// @throws maliput::common::assertion_error When @p road_network is nullptr.
  RoutePhase(
      int index,
      double lane_s_range_tolerance,
      const maliput::api::RoadPosition& start_road_position,
      const maliput::api::RoadPosition& end_road_position,
      const maliput::api::LaneSRange& default_lane_s_ranges,
      const std::vector<maliput::api::LaneSRange>& adjancent_lane_s_ranges,
      const maliput::api::RoadNetwork* road_network) :
        index_(index), must_switch_(must_switch), lane_s_range_tolerance_(lane_s_range_tolerance),
        default_lane_s_range_(default_lane_s_range), adjancent_lane_s_ranges_(adjancent_lane_s_ranges),
        road_network_(road_network) {
    MALIPUT_THROW_UNLESS(index_ >= 0);
    MALIPUT_THROW_UNLESS(lane_s_range_tolerance >= 0.);
    MALIPUT_THROW_UNLESS(road_network_ != nullptr);
    MALIPUT_THROW_UNLESS(start_road_position_.lane != nullptr);
    MALIPUT_THROW_UNLESS(end_road_position_.lane != nullptr);
    // TODO(#543): Validate that start_road_position_.lane->id() and position are in the default_lane_s_range and
    // adjacent_lane_s_ranges.
    // TODO(#543): Validate that start_road_position_.lane->id() and position are in the default_lane_s_range and
    // adjacent_lane_s_ranges.
    // TODO(#543): Validate that adjancent_lane_s_ranges_ are effectively adjacent to default_lane_s_range.
  }

  /// @return The index of this RoutePhase.
  int Index() const { return index_; }

  /// @return The start maliput::api::RoadPosition of this RoutePhase.
  const maliput::api::RoadPosition& StartRoadPosition() const { return start_road_position_; }

  /// @return The end maliput::api::RoadPosition of this RoutePhase.
  const maliput::api::RoadPosition& EndRoadPosition() const { return end_road_position_; }

  /// @return The default maliput::api::LaneSRange.
  const maliput::api::LaneSRange& DefaultLaneSRange() const { return default_lane_s_range_; }

  /// @return The maliput::api::rules::RoadRulebook::QueryResults for the
  /// default maliput::api::LaneSRange.
  maliput::api::rules::RoadRulebook::QueryResults RulesForDefaultLaneSRange() const {
    return road_network_->rulebook()->FindRules({default_lane_s_range_});
  }

  /// @return A vector of maliput::api::LaneSRanges with the adjacent
  /// maliput::api::LaneSRanges.
  std::vector<maliput::api::LaneSRange> AdjacentLaneSRanges() const { return adjancent_lane_s_ranges_; }

  /// Finds the rules for @p lane_s_range.
  ///
  /// The @p lane_s_range must intersect one of the default or adjacent
  /// maliput::api::LaneSRanges.
  /// (To be discussed): we probably want it to be included rather than intersected.
  ///
  /// @param lane_s_range The maliput::api::LaneSRange to consider.
  /// @return The rules that apply to that @p lane_s_range.
  /// @throws maliput::common::assertion_error When @p lane_s_range does not
  /// intersect any of the default or adjacent maliput::api::LaneSRanges.
  maliput::api::rules::RoadRulebook::QueryResults RulesForLaneSRange(
      const maliput::api::LaneSRange& lane_s_range) const;

  /// Finds the maliput::api::LaneSRange where @p inertial_position falls into.
  ///
  /// @param inertial_position An INERTIAL-Frame position.
  /// @return An optional with the maliput::api::LaneSRange when the RoutePhase
  /// contains a LaneSRange which holds @p inertial_position.
  std::optional<maliput::api::LaneSRange> FindLaneSRangeBy(
      const maliput::api::InertialPosition& inertial_position) const;

  /// Finds the maliput::api::LaneSRange where @p road_position falls into.
  ///
  /// @param road_position A LANE-Frame position.
  /// @return An optional with the maliput::api::LaneSRange when the RoutePhase
  /// contains a LaneSRange which holds @p road_position.
  std::optional<maliput::api::LaneSRange> FindLaneSRangeBy(
      const maliput::api::RoadPosition& road_position) const;

 private:
  int index_{};
  maliput::api::RoadPosition start_road_position_{};
  maliput::api::RoadPosition end_road_position_{};
  double lane_s_range_tolerance_{};
  maliput::api::LaneSRange default_lane_s_range_;
  std::vector<maliput::api::LaneSRange> adjacent_lane_s_ranges_{};
  const maliput::api::RoadNetwork* road_network_{};
};

}  // namespace routing
}  // namespace maliput
