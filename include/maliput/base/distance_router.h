// BSD 3-Clause License
//
// Copyright (c) 2024, Woven by Toyota. All rights reserved.
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
#include "maliput/routing/graph/graph.h"
#include "maliput/routing/route.h"
#include "maliput/routing/router.h"
#include "maliput/routing/routing_constraints.h"

namespace maliput {

/// Basic implementation of a Router which only looks at the travelled distance
/// to provide solutions.
///
/// The routing algorithm will consider the minimum length of api::LaneSRanges within
/// a routing::Phase as its cost. The accumulation of all routing::Phases' costs along
/// a routing::Route determines its cost.
// TODO: provide solutions that rely on segment-to-segment connectivity and enable
// the use of api::Lane switches in results.
class DistanceRouter : public routing::Router {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(DistanceRouter);

  /// Constructs a DistanceRouter.
  ///
  /// @param road_network The api::RoadNetwork to compute routing::Routes on. It must remain valid for the lifetime
  /// of the constructed object.
  /// @param lane_s_range_tolerance The tolerance to consider when evaluating api::LaneSRanges. It must not be negative.
  /// @throws common::assertion_error When @p lane_s_range_tolerance is negative.
  DistanceRouter(const api::RoadNetwork& road_network, double lane_s_range_tolerance);

 private:
  std::vector<routing::Route> DoComputeRoutes(const api::RoadPosition& start, const api::RoadPosition& end,
                                              const routing::RoutingConstraints& routing_constraints) const override;

  const api::RoadNetwork& road_network_;
  const double lane_s_range_tolerance_{};
  const routing::graph::Graph graph_{};
};

}  // namespace maliput
