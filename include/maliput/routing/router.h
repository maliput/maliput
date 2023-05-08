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

#include <optional>
#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/road_network.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/routing/route.h"
#include "maliput/routing/route_phase.h"
#include "maliput/routing/routing_constraints.h"

namespace maliput {
namespace routing {

/// Computes Routes that join one point with another in the
/// maliput::api::RoadGeometry.
///
/// Implementations of this class may integrate different policies to compute
/// Routes, such as length, maximum speed, driving allowance, etc. One or many
/// implementations can be created per agent type with different customizations
/// might yield different routes upon the same maliput::api::RoadGeometry and
/// set of arguments of your query. This is correct and expected behavior as the
/// particular implementation details must rule the decisions that build a
/// Route.
class Router {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Router);

  /// Computes the Route that joins @p start_road_postion to
  /// @p end_road_position under @p routing_constraints.
  ///
  /// @param start_road_position The start point in the maliput::api::RoadGeometry.
  /// It must be a valid maliput::api::RoadPosition within the maliput::api::RoadGeometry.
  /// @param end_road_position The end point in the maliput::api::RoadGeometry.
  /// It must be a valid maliput::api::RoadPosition within the maliput::api::RoadGeometry.
  /// @param routing_constraints The set of constraints that apply to the routing
  /// algorithm when computing the Route. It must be valid. @see ValidateRoutingConstraints().
  /// @return An optional with a Route that joins @p start_road_position with
  /// @p end_road_position under @p routing_constraints.
  /// @throws maliput::common::assertion_error When @p start_road_position is not valid.
  /// @throws maliput::common::assertion_error When @p end_road_position is not valid.
  /// @throws maliput::common::assertion_error When @p routing_constraints is not valid.
  std::optional<Route> ComputeRoute(const maliput::api::RoadPosition& start_road_position,
                                    const maliput::api::RoadPosition& end_road_position,
                                    const RoutingConstraints& routing_constraints) const {
    return DoComputeRoute(start_road_position, end_road_position, routing_constraints);
  }

  /// @return The parent maliput::api::RoadNetwork pointer.
  const maliput::api::RoadNetwork* road_network() const { return road_network_; }

 protected:
  // Implementations should call this constructor to set the @p road_network.
  //
  // @param road_network The maliput::api::RoadNetwork pointer. It must not be
  // nullptr. The lifetime of this pointer must exceed that of this object.
  // @throws maliput::common::assertion_error When @p road_network is nullptr.
  explicit Router(const maliput::api::RoadNetwork* road_network) : road_network_(road_network) {
    MALIPUT_THROW_UNLESS(road_network_ != nullptr);
  }

 private:
  virtual std::optional<Route> DoComputeRoute(const maliput::api::RoadPosition& start_road_position,
                                              const maliput::api::RoadPosition& end_road_position,
                                              const RoutingConstraints& routing_constraints) const = 0;

  const maliput::api::RoadNetwork* road_network_{};
};

}  // namespace routing
}  // namespace maliput
