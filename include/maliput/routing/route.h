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
#include "maliput/routing/lane_s_relation.h"
#include "maliput/routing/route_phase.h"
#include "maliput/routing/route_position_result.h"

namespace maliput {
namespace routing {

/// Describes the sequence of paths to go from one RoadPosition to another.
/// It is sequenced by RoutePhases which contain default and adjacent
/// maliput::api::LaneSRanges which an agent can use to travel from the start to
/// the end road position.
///
/// Agents are expected to use the Router to obtain a Route. Once in the Route,
/// they can iterate through the RoutePhases or find a specific RoutePhase via
/// an INERTIAL or LANE Frame coordinate and start driving from there towards
/// the end goal.
///
/// The first RoutePhase start road position identifies the beginning of the
/// Route. The last RoutePhase end road position identifies the ending of the
/// Route. The sequence of RoutePhases form a continuous route where the end of
/// one RoutePhase exactly matches the begining of the next RoutePhase in the
/// sequence.
class Route final {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Route);
  Route() = delete;

  /// Constructs a Route.
  ///
  /// @param route_phases The sequence of RoutePhases. It must not be empty.
  /// RoutePhases must be connected end to end.
  /// @param road_network The maliput::api::RoadNetwork pointer. It must not be
  /// nullptr. The lifetime of this pointer must exceed that of this object.
  /// @throws maliput::common::assertion_error When @p route_phases is empty.
  /// @throws maliput::common::assertion_error When @p route_phases contains
  /// RoutePhases that are not connected end to end, or contain locations not in
  /// @p road_network.
  /// @throws maliput::common::assertion_error When @p road_network is nullptr.
  Route(const std::vector<RoutePhase>& route_phases, const maliput::api::RoadNetwork* road_network)
      : route_phases_(route_phases), road_network_(road_network) {
    MALIPUT_THROW_UNLESS(!route_phases_.empty());
    MALIPUT_THROW_UNLESS(road_network_ != nullptr);
    /// TODO(#453): Validate end to end connection of the RoutePhases.
    /// TODO(#453): Validate RoutePhases are in the RoadNetwork.
  }

  /// @return The number of RoutePhases.
  int Size() const { return static_cast<int>(route_phases_.size()); }

  /// Indexes the RoutePhases.
  ///
  /// @param index The index of the RoutePhase. It must be non-negative and
  /// less than `size()`.
  /// @return The RoutePhase at @p index.
  /// @throws std::out_of_range When @p index is negative or >= `size()`.
  const RoutePhase& Get(int index) const { return route_phases_.at(index); }

  /// @return The start maliput::api::RoadPosition of this Route.
  const maliput::api::RoadPosition& StartRoadPosition() const { return route_phases_.front().StartRoadPosition(); }

  /// @return The end maliput::api::RoadPosition of this Route.
  const maliput::api::RoadPosition& EndRoadPosition() const { return route_phases_.back().EndRoadPosition(); }

  /// Finds the RoutePositionResult which @p inertial_position best fits.
  ///
  /// The fitting of the @p inertial_position into the complete Route will use
  /// the same set of rules maliput::api::RoadGeometry::ToRoadPosition() uses to
  /// find a matching maliput::api::RoadPositionResult within the maliput::api::RoadGeometry.
  /// When the @p inertial_position does not fall into the volume defined by the
  /// set of maliput:api::LaneSRanges each RoutePhase has, the returned
  /// RoutePhase will be the one that minimizes the Euclidean distance to the
  /// Route.
  /// The mapping is done right on `r=0, h=0` over the maliput::api::Lanes, i.e.
  /// at the centerline. This means that the returned `distance` and INERTIAL-
  /// Frame are evaluated there as well.
  ///
  /// @param inertial_position An INERTIAL-Frame position.
  /// @return A RoutePositionResult.
  RoutePositionResult FindRoutePositionBy(const maliput::api::InertialPosition& inertial_position) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

  /// Finds the RoutePositionResult which @p road_position best fits.
  ///
  /// The fitting of the @p road_position into the complete Route will use
  /// the same set of rules maliput::api::RoadGeometry::ToRoadPosition() uses to
  /// find a matching maliput::api::RoadPositionResult within the maliput::api::RoadGeometry.
  /// When the @p road_position does not fall into the volume defined by the
  /// set of maliput:api::LaneSRanges each RoutePhase has, the returned
  /// RoutePhase will be the one that minimizes the Euclidean distance to the
  /// Route.
  /// The mapping is done right on `r=0, h=0` over the maliput::api::Lanes, i.e.
  /// at the centerline. This means that the returned `distance` and INERTIAL-
  /// Frame are evaluated there as well.
  ///
  /// @param road_position A road position expressed into the LANE-Frame
  /// position. It must be valid.
  /// @return A RoutePositionResult.
  /// @throws maliput::common::assertion_error When @p road_position is not
  /// valid.
  RoutePositionResult FindRoutePositionBy(const maliput::api::RoadPosition& road_position) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

  /// Finds the relationship between @p lane_s_range_a and @p lane_s_range_b.
  ///
  /// @param lane_s_range_a A maliput::api::LaneSRange.
  /// @param lane_s_range_b A maliput::api::LaneSRange.
  /// @return The LaneSRangeRelation between @p lane_s_range_a and @p lane_s_range_b.
  LaneSRangeRelation LaneSRangeRelationFor(const maliput::api::LaneSRange& lane_s_range_a,
                                           const maliput::api::LaneSRange& lane_s_range_b) const {
    MALIPUT_THROW_MESSAGE("Unimplemented");
  }

 private:
  std::vector<RoutePhase> route_phases_{};
  const maliput::api::RoadNetwork* road_network_{};
};

}  // namespace routing
}  // namespace maliput
