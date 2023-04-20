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
#include "maliput/routing/route_phase.h"

namespace maliput {
namespace routing {

/// Describes the sequence of paths to go from one RoadPosition to another.
/// It is sequenced by RoutePhases which contain default and adjacent
/// maliput::api::LaneSRanges which conduce the agent from the start to the end
/// road position.
///
/// Agents are expected to use the Router to obtain a Route. Once in the Route,
/// they can iterate through the RoutePhases or find by a specific INERTIAL or
/// LANE Frame coordinate the RoutePhase and start driving from there towards
/// the end goal.
///
/// The first RoutePhase start road position identifies the beginning of the
/// Route. The last RoutePhase end road position identifies the ending of the
/// Route. RoutePhases in the sequence are connected end to end by the same
/// INERTIAL-Frame coordinate whose the corresponding LANE-Frame coordinate
/// depends on which maliput::api::LaneSRange they contain.
class Route final {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Route);
  Route() = delete;

  /// Constructs a Route.
  ///
  /// @param route_phases The sequence of RoutePhases. It must not be nullptr.
  /// RoutePhases must be connected end to end.
  /// @param road_network The maliput::api::RoadNetwork pointer. It must not be
  /// nullptr.
  /// @throws maliput::common::assertion_error When @p route_phases is empty.
  /// @throws maliput::common::assertion_error When @p route_phases contains
  /// RoutePhases that are not connected end to end.
  /// @throws maliput::common::assertion_error When @p road_networ is nullptr.
  Route(const std::vector<RoutePhases>& route_phases, const maliput::api::RoadNetwork* road_network) :
      route_phases_(route_phases), road_network_(road_network) {
    MALIPUT_THROW_UNLESS(!route_phases_.empty());
    MALIPUT_THROW_UNLESS(road_network_ != nullptr);
    /// TODO(#453): Validate end to end connection of the RoutePhases.
  }

  /// @return The number of RoutePhases.
  int Size() const { static_cast<int>(route_phases_.size()); }

  /// Indexes the RoutePhases.
  ///
  /// @param index The index of the RoutePhase. It must be not be negative and
  /// less than `size()`.
  /// @return The RoutePhase at @p index.
  /// @throws std::out_of_range When @p index is negative or >= `size()`.
  const RoutePhase& Get(int index) const { return route_phases_.at(index); }

  /// @return The start maliput::api::RoadPosition of this Route.
  const maliput::api::RoadPosition& StartRoadPosition() const {
    return route_phases_.front().StartRoadPosition();
  }

  /// @return The ebd maliput::api::RoadPosition of this Route.
  const maliput::api::RoadPosition& EndRoadPosition() const {
    return route_phases_.back().EndRoadPosition();
  }

  /// Finds the RoutePhase where @p inertial_position falls into.
  ///
  /// @param inertial_position An INERTIAL-Frame position.
  /// @return An optional with the RoutePhase which contains @p inertial_position.
  std::optional<RoutePhase> FindRoutePhaseBy(
      const maliput::api::InertialPosition& inertial_position) const;

  /// Finds the RoutePhase where @p road_position falls into.
  ///
  /// @param road_position A LANE-Frame position.
  /// @return An optional with the RoutePhase which contains @p road_position.
  std::optional<RoutePhase> FindRoutePhaseBy(
      const maliput::api::RoadPosition& road_position) const;

 private:
  std::vector<RoutePhases> route_phases_;
  const maliput::api::RoadNetwork* road_network_{};
};

}  // namespace routing
}  // namespace maliput
