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
#include "maliput/api/road_network.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/routing/lane_s_range_relation.h"
#include "maliput/routing/route_phase.h"
#include "maliput/routing/route_position_result.h"

namespace maliput {
namespace routing {

/// Describes the sequence of paths that go from one api::RoadPosition to
/// another.
///
/// It hosts a subset of the api::RoadGeometry that represents a valid routing
/// graph for agent navigation. The graph is represented by a sequence of
/// RoutePhases. These entities allow agents to reduce the path search space
/// by constraining the lookup towards the end api::RoadPosition.
///
/// Agents are expected to use the Router to obtain a Route. Once in the Route,
/// they can iterate through the RoutePhases or find a specific RoutePhase via
/// an INERTIAL or LANE Frame coordinate and start driving from there towards
/// the end goal.
///
/// The first RoutePhase start road position identifies the beginning of the
/// Route. The last RoutePhase end road position identifies the ending of the
/// Route. The sequence of RoutePhases form a continuous route where the end of
/// one RoutePhase exactly matches the beginning of the next RoutePhase in the
/// sequence.
///
/// A valid representation of this routing request for this geometry (starting
/// `s` and ending at `e`):
/// <pre>
///        S1       S2       S3      S4
///                                       e    L0
///                                     /  x   L1
///                                   /  /
///                                 /  /
/// L0 x--------x--------x--------x  /
/// L1 x--------x--------x--------x
/// L2 x--------x--------x--------x--------    L0
///            /
///          /                        S5
///        /
///      /
/// S0 s   L0
/// </pre>
///
/// And yields the following route:
/// <pre>
///        S1       S2       S3      S4
///                                       e    L0
///                                     // x   L1
///                                   // //
///                                 // // <-- RoutingPhase, index 3
/// L0 x--------x________*________*  //
/// L1 x--------x________*________*
/// L2 x--------*________*________x--------    L0
///            // ^        ^_ RoutingPhase, index 2
///          //    \                  S5
///        //        RoutingPhase, index 1
///      //   <-- RoutingPhase, index 0
/// S0 s   L0
/// </pre>
///
/// Where:
/// - `s` indicates the start of the Route.
/// - `e` indicates the end of the Route.
/// - `x` indicates the start of end of an api::LaneSRange.
/// - `*` indicates the start of end of RoutingPhase.
/// - `-` indicates the path of an api::LaneSRange.
/// - `_` indicates the path of an api::LaneSRange within a RoutingPhase.
/// - `S0`, `S1`, ...: are api::Segments.
/// - `L0`, `L1`, ...: are api::Lanes.
///
/// Depending on where queried, different api::LaneSRoutes are returned, for
/// example, if queried from `s`, the returned sequence will be:
/// {S0:L0, S2:L2, S2:L1, S2:L0, S3:L0, S4:L0}
///
/// And if queried at any point in S3:L2: {S3:L2, S3:L1, S3:L0, S4:L0}.
class Route final {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Route);
  Route() = delete;

  /// Constructs a Route.
  ///
  /// @param route_phases The sequence of RoutePhases. It must not be empty.
  /// RoutePhases must be connected end to end.
  /// @param road_network The api::RoadNetwork pointer. It must not be
  /// nullptr. The lifetime of this pointer must exceed that of this object.
  /// @throws common::assertion_error When @p route_phases is empty.
  /// @throws common::assertion_error When @p route_phases is not connected end
  /// to end.
  /// @throws common::assertion_error When @p road_network is nullptr.
  Route(const std::vector<RoutePhase>& route_phases, const api::RoadNetwork* road_network)
      : route_phases_(route_phases), road_network_(road_network) {
    MALIPUT_THROW_UNLESS(!route_phases_.empty());
    MALIPUT_THROW_UNLESS(road_network_ != nullptr);
    /// TODO(#453): Validate end to end connection of the RoutePhases.
  }

  /// @return The number of RoutePhases.
  int size() const { return static_cast<int>(route_phases_.size()); }

  /// Indexes the RoutePhases.
  ///
  /// @param index The index of the RoutePhase. It must be non-negative and
  /// less than `size()`.
  /// @return The RoutePhase at @p index.
  /// @throws std::out_of_range When @p index is negative or >= `size()`.
  const RoutePhase& Get(int index) const { return route_phases_.at(index); }

  /// @return The start of this Route.
  const api::RoadPosition& start_route_position() const { return route_phases_.front().start_positions().front(); }

  /// @return The end of this Route.
  const api::RoadPosition& end_route_position() const { return route_phases_.back().end_positions().front(); }

  /// Finds the RoutePositionResult which @p inertial_position best fits.
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
  /// Frame are evaluated there as well.
  ///
  /// @param inertial_position The INERTIAL-Frame position.
  /// @return A RoutePositionResult.
  RoutePositionResult FindRoutePositionBy(const api::InertialPosition& inertial_position) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

  /// Finds the RoutePositionResult which @p road_position best fits.
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
  /// Frame are evaluated there as well.
  ///
  /// @param road_position The road position. It must be valid.
  /// @return A RoutePositionResult.
  /// @throws common::assertion_error When @p road_position is not
  /// valid.
  RoutePositionResult FindRoutePositionBy(const api::RoadPosition& road_position) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

  /// Finds the relation between @p lane_s_range_b with respect to
  /// @p lane_s_range_a.
  ///
  /// @param lane_s_range_a An api::LaneSRange.
  /// @param lane_s_range_b An api::LaneSRange.
  /// @return The LaneSRangeRelation between @p lane_s_range_b with respect to
  /// @p lane_s_range_a.
  LaneSRangeRelation LaneSRangeRelationFor(const api::LaneSRange& lane_s_range_a,
                                           const api::LaneSRange& lane_s_range_b) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

  /// Computes an api::LaneSRoute from @p start_position towards
  /// end_route_position().
  ///
  /// The resulting api::LaneSRoute aims for reducing the number of lane
  /// switches.
  /// When @p start_position is not within this Route, it will be transformed by
  /// FindRoutePositionBy() to a point within it and then the api::LaneSRoute
  /// will be computed.
  ///
  /// @param start_position The start api::RoadPosition of this path. It must be
  /// valid.
  /// @return The api::LaneSRoute connecting @p start_position and
  /// end_route_position().
  /// @throws common::assertion_error When @p start_position is not valid.
  api::LaneSRoute ComputeLaneSRoute(const api::RoadPosition& start_position) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

 private:
  std::vector<RoutePhase> route_phases_;
  const api::RoadNetwork* road_network_{};
};

}  // namespace routing
}  // namespace maliput
