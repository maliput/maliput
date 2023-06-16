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

#include "maliput/api/lane_data.h"
#include "maliput/api/road_network.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/routing/route.h"
#include "maliput/routing/route_phase.h"
#include "maliput/routing/routing_constraints.h"

namespace maliput {
namespace routing {

/// Computes Routes within an api::RoadNetwork.
///
/// Implementations of this class may integrate different policies to compute
/// Routes, such as length, maximum speed, driving allowance, etc. One or many
/// implementations can be created per agent type with different customizations
/// might yield different routes upon the same api::RoadNetwork and
/// set of arguments of your query. This is correct and expected behavior as the
/// particular implementation details must rule the decisions that build a
/// Route.
class Router {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Router);

  /// Computes the Route that joins @p start to @p end under
  /// @p routing_constraints.
  ///
  /// @param start The start point in the api::RoadNetwork.
  /// It must be a valid api::RoadPosition within the api::RoadNetwork.
  /// @param end The end point in the api::RoadNetwork.
  /// It must be a valid api::RoadPosition within the api::RoadNetwork.
  /// @param routing_constraints The set of constraints that apply to the routing
  /// algorithm when computing the Route. It must be valid. @see ValidateRoutingConstraints().
  /// @return An optional with a Route that joins @p start with @p end under
  /// @p routing_constraints.
  /// @throws common::assertion_error When @p start is not valid.
  /// @throws common::assertion_error When @p end is not valid.
  /// @throws common::assertion_error When @p routing_constraints is not valid.
  std::optional<Route> ComputeRoute(const api::RoadPosition& start, const api::RoadPosition& end,
                                    const RoutingConstraints& routing_constraints) const {
    return DoComputeRoute(start, end, routing_constraints);
  }

 protected:
  Router() = default;

 private:
  virtual std::optional<Route> DoComputeRoute(const api::RoadPosition& start, const api::RoadPosition& end,
                                              const RoutingConstraints& routing_constraints) const = 0;
};

}  // namespace routing
}  // namespace maliput
