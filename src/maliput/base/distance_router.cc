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
#include "maliput/base/distance_router.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <utility>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/routing/derive_lane_s_routes.h"
#include "maliput/routing/find_lane_sequences.h"
#include "maliput/routing/phase.h"

namespace maliput {
namespace {

// Computes the cost of a routing::Phase.
// @param phase The routing::Phase to compute the cost.
// @return The minimum length across all api::LaneSRanges in the routing::Phase.
double ComputeCost(const routing::Phase& phase) {
  double cost{std::numeric_limits<double>::max()};
  for (const api::LaneSRange& lane_s_range : phase.lane_s_ranges()) {
    cost = std::min(cost, lane_s_range.length());
  }
  return cost;
}

// Computes the cost of a routing::Route.
// @param route The routing::Route to compute the cost.
// @return The accumulated cost of all the routing::Phases in @p route.
double ComputeCost(const routing::Route& route) {
  double cost{0.};
  for (int i = 0; i < route.size(); ++i) {
    cost += ComputeCost(route.Get(i));
  }
  return cost;
}

/// Determines the maximum cost of all routing::Phases in @p route.
/// @param route The routing::Route to look for the maximum routing::Phase's cost.
/// @return The maximum cost of all routing::Phases in @p route.
double MaxPhaseCostInRoute(const routing::Route& route) {
  double cost{0.};
  for (int i = 0; i < route.size(); ++i) {
    cost = std::max(ComputeCost(route.Get(i)), cost);
  }
  return cost;
}

// Filters routing::Routes from @p routes based on @p routing_constraints.
// @param routing_constraints The constraints to evaluate over @p routes.
// @param routes The collection of routing::Routes that connect the start and end.
// @return A subset of @p routes that comply with @p routing_constraints.
std::vector<routing::Route> FilterRoutes(const routing::RoutingConstraints& routing_constraints,
                                         const std::vector<routing::Route>& routes) {
  // TODO: evaluate routing_constraints.allow_lane_switch.
  auto filter = [routing_constraints](const routing::Route& route) -> bool {
    if (routing_constraints.max_phase_cost.has_value() &&
        routing_constraints.max_phase_cost.value() < MaxPhaseCostInRoute(route)) {
      return false;
    }
    if (routing_constraints.max_route_cost.has_value() &&
        routing_constraints.max_route_cost.value() < ComputeCost(route)) {
      return false;
    }
    return true;
  };
  std::vector<routing::Route> filtered_routes;
  std::copy_if(routes.begin(), routes.end(), std::back_inserter(filtered_routes), filter);
  return filtered_routes;
}

}  // namespace

DistanceRouter::DistanceRouter(const api::RoadNetwork* road_network, double lane_s_range_tolerance)
    : Router(), road_network_(road_network), lane_s_range_tolerance_(lane_s_range_tolerance) {
  MALIPUT_THROW_UNLESS(road_network_ != nullptr);
  MALIPUT_THROW_UNLESS(lane_s_range_tolerance_ >= 0.0);
}

std::vector<routing::Route> DistanceRouter::DoComputeRoutes(
    const api::RoadPosition& start, const api::RoadPosition& end,
    const routing::RoutingConstraints& routing_constraints) const {
  // Validate input parameters.
  MALIPUT_THROW_UNLESS(start.lane != nullptr);
  MALIPUT_THROW_UNLESS(road_network_->road_geometry()->ById().GetLane(start.lane->id()) == start.lane);
  MALIPUT_THROW_UNLESS(end.lane != nullptr);
  MALIPUT_THROW_UNLESS(road_network_->road_geometry()->ById().GetLane(end.lane->id()) == end.lane);
  routing::ValidateRoutingConstraints(routing_constraints);
  static constexpr bool kRemoveUTurns{true};

  // Obtain the lane sequences that connect the start with end.
  const std::vector<std::vector<const api::Lane*>> lane_sequences =
      routing::FindLaneSequences(start.lane, end.lane, std::numeric_limits<double>::max(), kRemoveUTurns);

  // Construct the routes.
  // TODO: lateral routing::Phase inflation.
  const double start_s = start.pos.s();
  const double end_s = end.pos.s();
  std::vector<routing::Route> routes;
  for (const std::vector<const api::Lane*>& lane_sequence : lane_sequences) {
    if (lane_sequence.size() == 1u) {
      const routing::Phase phase(0, lane_s_range_tolerance_, {start}, {end},
                                 {api::LaneSRange(start.lane->id(), api::SRange(start_s, end_s))}, road_network_);
      routes.emplace_back(std::vector<routing::Phase>{phase}, road_network_);
      continue;
    }

    // Handles the case when lane_sequence has a length greater than 1.
    std::vector<routing::Phase> phases;
    for (int i = 0; i < static_cast<int>(lane_sequence.size()); ++i) {
      const api::Lane* lane = lane_sequence[i];
      if (i == 0) {
        const std::optional<double> first_end_s = routing::DetermineEdgeS(lane, lane_sequence[1]);
        MALIPUT_THROW_UNLESS(first_end_s.has_value());
        phases.emplace_back(
            i, lane_s_range_tolerance_, std::vector<api::RoadPosition>{start},
            std::vector<api::RoadPosition>{api::RoadPosition(lane, api::LanePosition(*first_end_s, 0., 0.))},
            std::vector<api::LaneSRange>{api::LaneSRange(lane->id(), api::SRange(start_s, *first_end_s))},
            road_network_);
      } else if (i + 1 == static_cast<int>(lane_sequence.size())) {
        MALIPUT_THROW_UNLESS(lane->id() == end.lane->id());
        MALIPUT_THROW_UNLESS(i > 0);
        const std::optional<double> last_start_s = routing::DetermineEdgeS(lane, lane_sequence[i - 1]);
        MALIPUT_THROW_UNLESS(last_start_s.has_value());
        phases.emplace_back(
            i, lane_s_range_tolerance_,
            std::vector<api::RoadPosition>{api::RoadPosition(lane, api::LanePosition(*last_start_s, 0., 0.))},
            std::vector<api::RoadPosition>{api::RoadPosition(lane, api::LanePosition(end_s, 0., 0.))},
            std::vector<api::LaneSRange>{api::LaneSRange(lane->id(), api::SRange(*last_start_s, end_s))},
            road_network_);
      } else {
        const std::optional<double> middle_start_s = routing::DetermineEdgeS(lane, lane_sequence[i - 1]);
        const std::optional<double> middle_end_s = routing::DetermineEdgeS(lane, lane_sequence[i + 1]);
        MALIPUT_THROW_UNLESS(middle_start_s.has_value() && middle_end_s.has_value());
        phases.emplace_back(
            i, lane_s_range_tolerance_,
            std::vector<api::RoadPosition>{api::RoadPosition(lane, api::LanePosition(*middle_start_s, 0., 0.))},
            std::vector<api::RoadPosition>{api::RoadPosition(lane, api::LanePosition(*middle_end_s, 0., 0.))},
            std::vector<api::LaneSRange>{api::LaneSRange(lane->id(), api::SRange(*middle_start_s, *middle_end_s))},
            road_network_);
      }
    }
    routes.emplace_back(phases, road_network_);
  }

  return FilterRoutes(routing_constraints, routes);
}

}  // namespace maliput
