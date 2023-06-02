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
#include "maliput/common/maliput_throw.h"
#include "maliput/routing/route_position_result.h"

namespace maliput {
namespace routing {

/// Manages a phase in a complete Route, towards its completion of a Route.
///
/// It is composed of a default api::LaneSRange which is the corridor
/// the agent traversing the Route should follow. Alternative parallel adjacent
/// api.:LaneSRanges might be taken when provided.
///
/// When the start and end api::RoadPositions belong to different
/// api::Lanes, a switch of api::Lanes is required.
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
  /// @param index The index at the parent Route. It must be non-negative.
  /// @param lane_s_range_tolerance Tolerance to compare api::LaneSRanges.
  /// It must be non-negative.
  /// @param start_road_position The start api::RoadPosition of this
  /// RoutePhase. `start_road_position.lane` must not be nullptr and it must be
  /// in @p lane_s_ranges.
  /// @param end_road_position The end api::RoadPosition of this
  /// RoutePhase. `end_road_position.lane` must not be nullptr and it must be
  /// in @p lane_s_ranges.
  /// @param lane_s_ranges List of api::LaneSRanges. It must not be empty, all
  /// elements must exist in @p road_network and should be consecutive adjacent.
  /// @param default_lane_s_range_index The index of the default api::LaneSRange
  /// in @p lane_s_ranges. It must be a valid index.
  /// @param road_network The pointer to the api::RoadNetwork. It must
  /// not be nullptr. The lifetime of this pointer must exceed that of this
  /// object.
  /// @throws common::assertion_error When @p index is negative.
  /// @throws common::assertion_error When @p lane_s_range_tolerance is
  /// negative.
  /// @throws common::assertion_error When @p start_road_position.lane is
  /// nullptr.
  /// @throws common::assertion_error When @p end_road_position.lane is
  /// nullptr.
  /// @throws common::assertion_error When @p lane_s_ranges is empty.
  /// @throws common::assertion_error When @p lane_s_ranges contains
  /// non-adjacent consecutive api::LaneSRanges.
  /// @throws common::assertion_error When @p lane_s_ranges contains
  /// api::LaneSRanges that do not exist in @p road_network.   
  /// @throws common::assertion_error When @p default_lane_s_range_index is not
  /// a valid index of @p lane_s_ranges.
  /// @throws common::assertion_error When @p road_network is nullptr.
  RoutePhase(int index,
             double lane_s_range_tolerance,
             const api::RoadPosition& start_road_position,
             const api::RoadPosition& end_road_position,
             const std::vector<api::LaneSRange>& lane_s_ranges,
             int default_lane_s_range_index,
             const api::RoadNetwork* road_network)
      : index_(index),
        lane_s_range_tolerance_(lane_s_range_tolerance),
        lane_s_ranges_(lane_s_ranges),
        default_lane_s_range_index_(default_lane_s_range_index),
        road_network_(road_network) {
    MALIPUT_THROW_UNLESS(index_ >= 0);
    MALIPUT_THROW_UNLESS(lane_s_range_tolerance >= 0.);    
    MALIPUT_THROW_UNLESS(start_road_position_.lane != nullptr);
    MALIPUT_THROW_UNLESS(end_road_position_.lane != nullptr);
    MALIPUT_THROW_UNLESS(!lane_s_ranges_.empty());
    MALIPUT_THROW_UNLESS(default_lane_s_range_index_ >= 0 && default_lane_s_range_index_ < static_cast<int>(lane_s_ranges_.size()));
    MALIPUT_THROW_UNLESS(road_network_ != nullptr);
    // TODO(#543): Validate that start_road_position_.lane->id() and position, or end_road_position_.lane->id() and position
    // are in the default api::LaneSRange of lane_s_ranges_.
    // TODO(#543): Validate that lane_s_ranges_ are effectively adjacent one to another.
    // TODO(#543): Validate api::LaneSRanges are in the RoadNetwork.
  }

  /// @return The index of this RoutePhase.
  int index() const { return index_; }

  /// @return The start api::RoadPosition of this RoutePhase.
  const api::RoadPosition& start_road_position() const {
    return start_road_position_;
  }

  /// @return The end api::RoadPosition of this RoutePhase.
  const api::RoadPosition& end_road_position() const {
    return end_road_position_;
  }

  /// @return The default api::LaneSRange.
  const api::LaneSRange& default_lane_s_range() const {
    return lane_s_ranges_.at(default_lane_s_range_index_);
  }

  /// @return The vector of api::LaneSRanges.
  const std::vector<api::LaneSRange>& lane_s_ranges() const {
    return lane_s_ranges_;
  }

  /// Finds the RoutePositionResult where @p inertial_position best fits.
  ///
  /// The fitting of the @p inertial_position into the complete Route will use
  /// the same set of rules api::RoadGeometry::ToRoadPosition() uses to
  /// find a matching api::RoadPositionResult within the api::RoadGeometry.
  /// When the @p inertial_position does not fall into the volume defined by the
  /// set of api::LaneSRanges each RoutePhase has, the returned
  /// RoutePhase will be the one that minimizes the Euclidean distance to the
  /// Route.
  /// The mapping is done right on `r=0, h=0` over the api::Lanes, i.e.
  /// at the centerline. This means that the returned `distance` and INERTIAL-
  /// Frame position are evaluated there as well.
  ///
  /// @param inertial_position The INERTIAL-Frame position.
  /// @return A RoutePositionResult.
  RoutePositionResult FindRoutePhasePositionBy(const api::InertialPosition& inertial_position) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

  /// Finds the RoutePositionResult where @p road_position best fits.
  ///
  /// The fitting of the @p road_position into the complete Route will use
  /// the same set of rules api::RoadGeometry::ToRoadPosition() uses to
  /// find a matching api::RoadPositionResult within the api::RoadGeometry.
  /// When the @p road_position does not fall into the volume defined by the
  /// set of api::LaneSRanges each RoutePhase has, the returned
  /// RoutePhase will be the one that minimizes the Euclidean distance to the
  /// Route.
  /// The mapping is done right on `r=0, h=0` over the api::Lanes, i.e.
  /// at the centerline. This means that the returned `distance` and INERTIAL-
  /// Frame position are evaluated there as well.
  ///
  /// @param road_position The road position. It must be valid.
  /// @return A RoutePositionResult.
  /// @throws common::assertion_error When @p road_position is not
  /// valid.
  RoutePositionResult FindRoutePhasePositionBy(const api::RoadPosition& road_position) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

 private:
  int index_{};
  double lane_s_range_tolerance_{};
  api::RoadPosition start_road_position_;
  api::RoadPosition end_road_position_;
  std::vector<api::LaneSRange> lane_s_ranges_;
  int default_lane_s_range_index_{};
  const api::RoadNetwork* road_network_{};
};

}  // namespace routing
}  // namespace maliput
