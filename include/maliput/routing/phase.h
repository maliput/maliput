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
#pragma once

#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_network.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/routing/route_position_result.h"

namespace maliput {
namespace routing {

/// Manages a phase in a Route, towards its end.
///
/// It is composed of a set of api::LaneSRanges. All these api::LaneSRanges
/// are adjacent and present different alternative path segments towards the end
/// of the Route. This is a convenient entity to reduce the result space of all
/// the api::LaneSRange permutations that yield valid paths within the Route.
///
/// All the initial api::RoadPositions constitute the set of start positions of
/// this phase. All the end api::RoadPositions constitute the set of end
/// positions of this phase. Certain api::RoadPositions in the start and end
/// sets may not have connectivity at the api::BranchPoint level, but the
/// api::LaneSRanges are included because they offer alternative volume for
/// agents to maneuver in. At least one api::RoadPosition in both the start and
/// end sets must be equal or overlapping in the INERTIAL-Frame another
/// api::RoadPosition in the preceeding and succeeding Phase respectively.
///
/// Agents can localize themselves within a Phase by using FindLaneSRange()
/// methods. This is useful when they are initially placing themselves on a path
/// or for iterative querying.
///
/// Simple agents may consider using directly the Route object and
/// Route::ComputeLaneSRoute() to traverse the Route. Those willing to exploit the
/// possibilities of the Route in favor of satisfying complex behaviors will find Phases
/// useful to better cluster adjacent api::LaneSRanges and the entry / exit
/// api::RoadPositions of them.
class Phase final {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Phase);
  Phase() = delete;

  /// Constructs a Phase.
  ///
  /// @param index The index at the parent Route. It must be non-negative.
  /// @param lane_s_range_tolerance Tolerance to compare api::LaneSRanges.
  /// It must be non-negative.
  /// @param start_positions The start api::RoadPositions of this
  /// Phase. Each api::RoadPosition must be valid and it must be in
  /// @p lane_s_ranges. There must be at least one api::RoadPosition.
  /// @param end_position The end api::RoadPositions of this
  /// Phase. Each api::RoadPosition must be valid and it must be in
  /// @p lane_s_ranges. There must be at least one api::RoadPosition.
  /// @param lane_s_ranges A right-to-left (see api::Segment semantics) ordered
  /// list of api::LaneSRanges. It must not be empty, all elements must exist
  /// in @p road_network and should be consecutively adjacent and ordered.
  /// @param road_network The pointer to the api::RoadNetwork. It must
  /// not be nullptr. The lifetime of this pointer must exceed that of this
  /// object.
  /// @throws common::assertion_error When @p index is negative.
  /// @throws common::assertion_error When @p lane_s_range_tolerance is
  /// negative.
  /// @throws common::assertion_error When any @p start_positions is
  /// invalid.
  /// @throws common::assertion_error When @p start_positions is empty.
  /// @throws common::assertion_error When any @p start_positions are not
  /// positions in @p lane_s_ranges.
  /// @throws common::assertion_error When any @p end_positions is
  /// invalid.
  /// @throws common::assertion_error When @p end_positions is empty.
  /// @throws common::assertion_error When any @p end_positions are not
  /// positions in @p lane_s_ranges.
  /// @throws common::assertion_error When @p lane_s_ranges is empty.
  /// @throws common::assertion_error When @p lane_s_ranges contains
  /// non-adjacent or ordered consecutive api::LaneSRanges.
  /// @throws common::assertion_error When @p lane_s_ranges contains
  /// api::LaneSRanges that do not exist in @p road_network.
  /// @throws common::assertion_error When @p road_network is nullptr.
  Phase(int index, double lane_s_range_tolerance, const std::vector<api::RoadPosition>& start_positions,
        const std::vector<api::RoadPosition>& end_positions, const std::vector<api::LaneSRange>& lane_s_ranges,
        const api::RoadNetwork* road_network);

  /// @return The index of this Phase.
  int index() const { return index_; }

  /// @return Tolerance to compare api::LaneSRanges.
  double lane_s_range_tolerance() const { return lane_s_range_tolerance_; }

  /// @return The start api::RoadPositions of this Phase.
  const std::vector<api::RoadPosition>& start_positions() const { return start_positions_; }

  /// @return The end api::RoadPositions of this Phase.
  const std::vector<api::RoadPosition>& end_positions() const { return end_positions_; }

  /// @return The vector of api::LaneSRanges.
  const std::vector<api::LaneSRange>& lane_s_ranges() const { return lane_s_ranges_; }

  /// Finds the PhasePositionResult where @p inertial_position best fits.
  ///
  /// The fitting of the @p inertial_position into the complete Route will use
  /// the same set of rules api::RoadGeometry::ToRoadPosition() uses to
  /// find a matching api::RoadPositionResult within the api::RoadGeometry.
  /// When the @p inertial_position does not fall into the volume defined by the
  /// set of api::LaneSRanges each Phase has, the returned
  /// Phase will be the one that minimizes the Euclidean distance to the
  /// Route.
  /// The mapping is done right on `r=0, h=0` over the api::Lanes, i.e.
  /// at the centerline. This means that the returned `distance` and INERTIAL-
  /// Frame position are evaluated there as well.
  ///
  /// @param inertial_position The INERTIAL-Frame position.
  /// @return A PhasePositionResult.
  PhasePositionResult FindPhasePosition(const api::InertialPosition& inertial_position) const;

  /// Finds the PhasePositionResult where @p road_position best fits.
  ///
  /// The fitting of the @p road_position into the complete Route will use
  /// the same set of rules api::RoadGeometry::ToRoadPosition() uses to
  /// find a matching api::RoadPositionResult within the api::RoadGeometry.
  /// When the @p road_position does not fall into the volume defined by the
  /// set of api::LaneSRanges each Phase has, the returned
  /// Phase will be the one that minimizes the Euclidean distance to the
  /// Route.
  /// The mapping is done right on `r=0, h=0` over the api::Lanes, i.e.
  /// at the centerline. This means that the returned `distance` and INERTIAL-
  /// Frame position are evaluated there as well.
  ///
  /// @param road_position The road position. It must be valid.
  /// @return A PhasePositionResult.
  /// @throws common::assertion_error When @p road_position is not
  /// valid.
  PhasePositionResult FindPhasePosition(const api::RoadPosition& road_position) const;

 private:
  int index_{};
  double lane_s_range_tolerance_{};
  std::vector<api::RoadPosition> start_positions_;
  std::vector<api::RoadPosition> end_positions_;
  std::vector<api::LaneSRange> lane_s_ranges_;
  const api::RoadNetwork* road_network_{};
};

/// Determines whether the @p position resides in any of @p lane_s_ranges.
///
/// @p tolerance is used to expand the range of @p lane_s_ranges when evaluating
/// @p position.
///
/// @param position The api::RoadPosition to evaluate. It must be valid.
/// @param lane_s_ranges The api::LaneSRanges. It must not be empty.
/// @param tolerance Tolerance to compare api::LaneSRanges' ranges with
/// @p position. It must be non-negative.
/// @return true When @p position is in any of @p lane_s_ranges with
/// @p tolerance in the LANE-Frame s coordinate.
/// @throws common::assertion_error When @p position is not valid.
/// @throws common::assertion_error When @p lane_s_ranges is empty.
/// @throws common::assertion_error When @p tolerance is negative.
bool ValidatePositionIsInLaneSRanges(const maliput::api::RoadPosition& position,
                                     const std::vector<api::LaneSRange>& lane_s_ranges, double tolerance);

}  // namespace routing
}  // namespace maliput
