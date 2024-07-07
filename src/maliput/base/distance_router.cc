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
#include "maliput/base/distance_router.h"

#include <algorithm>
#include <iterator>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/routing/derive_lane_s_routes.h"
#include "maliput/routing/find_lane_sequences.h"
#include "maliput/routing/graph/graph_utils.h"
#include "maliput/routing/phase.h"

namespace maliput {
namespace {

// Computes the cost of a routing::Phase.
//
// Treats all api::LaneSRanges within @p phase the same. Subsequent steps in the
// DistanceRouter will refine this cost estimate by, for example, considering
// which api::LaneSRanges will require lane changes to reach the route's end
// location.
//
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
//
// @param route The routing::Route to compute the cost.
// @return The accumulated cost of all the routing::Phases in @p route.
double ComputeCost(const routing::Route& route) {
  double cost{0.};
  for (int i = 0; i < route.size(); ++i) {
    cost += ComputeCost(route.Get(i));
  }
  return cost;
}

// Determines the maximum cost of all routing::Phases in @p route.
//
// @param route The routing::Route to look for the maximum routing::Phase's cost.
// @return The maximum cost of all routing::Phases in @p route.
double MaxPhaseCostInRoute(const routing::Route& route) {
  double cost{0.};
  for (int i = 0; i < route.size(); ++i) {
    cost = std::max(ComputeCost(route.Get(i)), cost);
  }
  return cost;
}

// Whether the agent driving the @p route is required to switch lanes to traverse the route.
//
// @param route To evaluate whether it has lane switches.
// @return true When @p route requires the agent traversing it to switch lanes.
bool HasLaneSwitch(const routing::Route& route) {
  // Look for the simple case where the routing::Route has only one routing::Phase and the start
  // and end positions are on the same api::Lane.
  if (route.size() == 1) {
    return route.start_route_position().lane != route.end_route_position().lane;
  }
  // Traverse the routing::Phases looking for lane switches.
  auto find_position_index = [](const std::vector<api::RoadPosition>& positions,
                                const api::Lane* lane) -> std::optional<size_t> {
    for (size_t i = 0; i < positions.size(); i++) {
      if (positions[i].lane == lane) {
        return i;
      }
    }
    return std::nullopt;
  };
  const api::Lane* start_lane = route.start_route_position().lane;
  for (int phase_index = 0; phase_index < route.size(); phase_index++) {
    const routing::Phase& phase = route.Get(phase_index);
    // Look in the end positions of the current routing::Phase for a position in the same api::Lane.
    // When there is no end position on the same start api::Lane, there is a lane switch within the routing::Phase.
    const std::optional<size_t> end_position_index = find_position_index(phase.end_positions(), start_lane);
    if (!end_position_index.has_value()) {
      return true;
    }
    // Update the start_lane for the next iteration.
    if (phase_index != route.size() - 1) {
      start_lane = route.Get(phase_index + 1).start_positions()[end_position_index.value()].lane;
    }
  }
  return false;
}

// Filters @p routes based on @p routing_constraints.
//
// @param routing_constraints The constraints to evaluate over @p routes.
// @param routes The collection of routing::Routes that connect the start and end.
// @return A subset of @p routes that comply with @p routing_constraints. The relative order in the returned
// routes is the same as @p routes.
std::vector<routing::Route> FilterRoutes(const routing::RoutingConstraints& routing_constraints,
                                         const std::vector<routing::Route>& routes) {
  auto filter = [routing_constraints](const routing::Route& route) -> bool {
    if (routing_constraints.max_phase_cost.has_value() &&
        routing_constraints.max_phase_cost.value() < MaxPhaseCostInRoute(route)) {
      return false;
    }
    if (routing_constraints.max_route_cost.has_value() &&
        routing_constraints.max_route_cost.value() < ComputeCost(route)) {
      return false;
    }
    if (!routing_constraints.allow_lane_switch && HasLaneSwitch(route)) {
      return false;
    }
    return true;
  };
  std::vector<routing::Route> filtered_routes;
  std::copy_if(routes.begin(), routes.end(), std::back_inserter(filtered_routes), filter);
  return filtered_routes;
}

// Concatenates items in @p errors into just one string.
//
// When @p errors is empty an empty string is computed.
// Otherwise, the "Route has connectivity errors: <error> | <error> | ... |" formula to concatenate items from @p errors
// is used.
// @param errors A vector of error strings.
// @return An empty string when @p errors is empty, otherwise a string with all the items in @p errors.
std::string AggregateRouteConnectivityErrors(const std::vector<std::string>& errors) {
  std::string result{errors.empty() ? "" : "Route has connectivity errors: "};
  for (const std::string& error : errors) {
    result += error + " | ";
  }
  return result;
}

// Iterates over @p routes and validates none have end to end connectivity errors.
//
// @param routes Vector of Routes to validate.
// @throws common::assertion_error When one of the Routes in @p routes has end to end connectivity errors.
void ValidateEndToEndConnectivityInRoutes(const std::vector<routing::Route>& routes) {
  for (const auto& route : routes) {
    const std::vector<std::string> connectivity_errors = route.ValidateEndToEndConnectivity();
    MALIPUT_VALIDATE(connectivity_errors.empty(), AggregateRouteConnectivityErrors(connectivity_errors));
  }
}

// Finds an equivalent s-coordinate from @p ref_pos in the target @p lane.
//
// The equivalence is derived by linearly scaling @p ref_pos's s-coordinate by the ratio of api::Lane::length().
// Uses @p tolerance to round up / down the equivalent s-coordinate to 0. or the @p lane's length.
// @param ref_pos The reference api::RoadPosition.
// @param lane The target api::Lane.
// @param tolerance The tolerance to adjust the scaled s-coordinate.
// @return The scaled s-coordinate.
double FindEquivalentSCoordinate(const api::RoadPosition& ref_pos, const api::Lane& lane, double tolerance) {
  const double ref_length = ref_pos.lane->length();
  const double target_length = lane.length();
  const double target_s = std::clamp(ref_pos.pos.s() / ref_length * target_length, 0., target_length);
  if (target_s <= tolerance && target_length > tolerance) {
    return 0.;
  }
  if ((target_length - target_s) <= tolerance && target_length > tolerance) {
    return target_length;
  }
  return target_s;
}

// Makes a routing::Phase.
//
// Computes an api::LaneSRange out of each api::Lane in @p edge.segment whose start and end positions are
// derived from the use of FindEquivalentSCoordinate() and the first element in @p start_positions and @p end_positions
// respectively.
// Arguments are mapped one to one to the routing::Phase constructor.
//
// @param index See routing::Phase() documentation.
// @param edge A routing::graph::Edge to compute the api::LaneSRanges for.
// @param lane_s_range_tolerance See routing::Phase() documentation.
// @param start_positions See routing::Phase() documentation.
// @param end_positions See routing::Phase() documentation.
// @param road_network See routing::Phase() documentation.
// @return A routing::Phase.
// @throws common::assertion_error When @p edge.segment is nullptr.
// @throws common::assertion_error When any of @p start_positions and @p end_positions are empty.
// @throws common::assertion_error When positions in @p start_positions and @p end_positions are not in @p edge.segment.
routing::Phase MakePhase(int index, const routing::graph::Edge& edge, double lane_s_range_tolerance,
                         const std::vector<api::RoadPosition>& start_positions,
                         const std::vector<api::RoadPosition>& end_positions, const api::RoadNetwork& road_network) {
  MALIPUT_THROW_UNLESS(edge.segment != nullptr);
  MALIPUT_THROW_UNLESS(!start_positions.empty());
  MALIPUT_THROW_UNLESS(!end_positions.empty());
  for (const auto& pos : start_positions) {
    MALIPUT_THROW_UNLESS(pos.lane->segment() == edge.segment);
  }
  for (const auto& pos : end_positions) {
    MALIPUT_THROW_UNLESS(pos.lane->segment() == edge.segment);
  }

  std::vector<api::LaneSRange> lane_s_ranges;
  for (int i = 0; i < edge.segment->num_lanes(); ++i) {
    const api::Lane* lane = edge.segment->lane(i);
    const double start_s = FindEquivalentSCoordinate(start_positions.front(), *lane, lane_s_range_tolerance);
    const double end_s = FindEquivalentSCoordinate(end_positions.front(), *lane, lane_s_range_tolerance);
    lane_s_ranges.emplace_back(lane->id(), api::SRange(start_s, end_s));
  }
  return routing::Phase{index, lane_s_range_tolerance, start_positions, end_positions, lane_s_ranges, &road_network};
}

// Computes all api::RoadPositions for @p ref ref_edge at @p ref_end side when connecting with @p target_edge.
//
// @param ref_edge The reference routing::graph::Edge to compute the api::RoadPositions for.
// @param ref_end Indicates api::LaneEnd::Which side of the api::Lanes from @p ref_edge to take the positions.
// @param target_edge The connecting routing::graph::Edge.
// @return A vector with the api::RoadPositions to that connect @p ref_edge with @p target_edge at @p ref_end side.
// @throws common::assertion_error When the return vector is empty.
std::vector<api::RoadPosition> ComputeAllRoadPositions(const routing::graph::Edge& ref_edge,
                                                       const api::LaneEnd::Which ref_end,
                                                       const routing::graph::Edge& target_edge) {
  auto is_lane_connected = [segment = target_edge.segment, end = ref_end](const api::Lane* lane,
                                                                          bool is_confluent) -> bool {
    const api::LaneEndSet* lane_end_set =
        is_confluent ? lane->GetConfluentBranches(end) : lane->GetOngoingBranches(end);
    for (int i = 0; i < lane_end_set->size(); ++i) {
      const api::LaneEnd lane_end = lane_end_set->get(i);
      if (lane_end.lane->segment() == segment) {
        return true;
      }
    }
    return false;
  };
  auto process_positions = [segment = ref_edge.segment, end = ref_end, is_lane_connected](bool is_confluent) {
    std::vector<api::RoadPosition> positions;
    for (int i = 0; i < segment->num_lanes(); ++i) {
      const api::Lane* lane = segment->lane(i);
      if (is_lane_connected(lane, is_confluent)) {
        positions.emplace_back(lane,
                               api::LanePosition{end == api::LaneEnd::Which::kStart ? 0. : lane->length(), 0., 0.});
      }
    }
    return positions;
  };
  constexpr bool kIsConfluent{true};
  constexpr bool kIsOngoing{!kIsConfluent};
  std::vector<api::RoadPosition> positions = process_positions(kIsConfluent);
  if (positions.empty()) {
    positions = process_positions(kIsOngoing);
  }
  MALIPUT_THROW_UNLESS(!positions.empty());
  return positions;
}

// Optionally introduces routing::graph::Edges at the beginning or end of @p edge_sequence to make sure @p start and
// @p end are included in it.
//
// Points fall always into routing::graph::Edges and the routing query requires an entry and exit routing::graph::Node.
// Because there is no notion of direction of travel, i.e. routing::graph::Edges are not directed, the paths may miss
// the edge that contains @p start and / or @p end. This function will introduce the missing routing::graph::Edge at
// the beginning and end of the sequence in @p edge_sequences when they are missing.
//
// @param graph The routing::graph::Graph to extract the routing::graph::Edges containing @p start and @p end.
// @param start The start api::RoadPosition. It must be valid.
// @param end The end api::RoadPosition. It must be valid.
// @param edge_sequences The vector of routing::graph::Edge sequences to amend.
// @return A vector of routing::graph::Edge sequences of the same size as @p edge_sequences whose sequences have at
// the beginning and at the end a routing::graph::Edge that contains @p start and @p end points respectively.
// @throws common::assertion_error When @p start or @p end are invalid api::RoadPositions.
std::vector<std::vector<routing::graph::Edge>> MaybeAddStartAndEndEdges(
    const routing::graph::Graph& graph, const api::RoadPosition& start, const api::RoadPosition& end,
    const std::vector<std::vector<routing::graph::Edge>>& edge_sequences) {
  MALIPUT_THROW_UNLESS(start.lane != nullptr);
  MALIPUT_THROW_UNLESS(end.lane != nullptr);

  const routing::graph::EdgeId start_edge_id(start.lane->segment()->id().string());
  const routing::graph::EdgeId end_edge_id(end.lane->segment()->id().string());
  const routing::graph::Edge& start_edge = graph.edges.at(start_edge_id);
  const routing::graph::Edge& end_edge = graph.edges.at(end_edge_id);

  std::vector<std::vector<routing::graph::Edge>> result;

  for (const auto& edge_sequence : edge_sequences) {
    std::vector<routing::graph::Edge> edge_sequence_to_return(edge_sequence);
    // When edge_sequence is empty, the start and end routing::graph::Edge are the same thus we simply add the
    // first one as if the edge is missing.
    if (edge_sequence.empty() || edge_sequence.front().segment != start_edge.segment) {
      edge_sequence_to_return.insert(edge_sequence_to_return.begin(), start_edge);
    }
    // Adds the end routing::graph::Edge when it is missing.
    if (edge_sequence.back().segment != end_edge.segment) {
      edge_sequence_to_return.push_back(end_edge);
    }
    result.push_back(edge_sequence_to_return);
  }

  return result;
}

}  // namespace

DistanceRouter::DistanceRouter(const api::RoadNetwork& road_network, double lane_s_range_tolerance)
    : Router(),
      road_network_(road_network),
      lane_s_range_tolerance_(lane_s_range_tolerance),
      graph_(routing::graph::BuildGraph(road_network_.road_geometry())) {
  MALIPUT_THROW_UNLESS(lane_s_range_tolerance_ >= 0.0);
}

std::vector<routing::Route> DistanceRouter::DoComputeRoutes(
    const api::RoadPosition& start, const api::RoadPosition& end,
    const routing::RoutingConstraints& routing_constraints) const {
  // Validate input parameters.
  MALIPUT_THROW_UNLESS(start.lane != nullptr);
  MALIPUT_THROW_UNLESS(road_network_.road_geometry()->ById().GetLane(start.lane->id()) == start.lane);
  MALIPUT_THROW_UNLESS(end.lane != nullptr);
  MALIPUT_THROW_UNLESS(road_network_.road_geometry()->ById().GetLane(end.lane->id()) == end.lane);
  routing::ValidateRoutingConstraints(routing_constraints);

  // Obtain the start and end routing::graph::Nodes in the graph_.
  const std::optional<routing::graph::Node> start_node =
      routing::graph::FindNode(graph_, *(start.lane), api::LaneEnd::Which::kStart);
  const std::optional<routing::graph::Node> end_node =
      routing::graph::FindNode(graph_, *(end.lane), api::LaneEnd::Which::kFinish);
  MALIPUT_THROW_UNLESS(start_node.has_value());
  MALIPUT_THROW_UNLESS(end_node.has_value());

  // Obtain the sequences of routing::graph::Edges that connect the start with end, i.e. the routing algorithm.
  std::vector<std::vector<routing::graph::Edge>> edge_sequences =
      routing::graph::FindAllEdgeSequences(graph_, start_node.value(), end_node.value());

  // Amends the routing::graph::Edge sequences by introducing the start and end routing::graph::Edges when
  // missing due to the direction of travel.
  edge_sequences = MaybeAddStartAndEndEdges(graph_, start, end, edge_sequences);

  std::vector<routing::Route> routes;
  for (const std::vector<routing::graph::Edge>& edge_sequence : edge_sequences) {
    // Handles the case when edge_sequence has a length of 1. This implies the route only has one routing::Phase.
    if (edge_sequence.size() == 1u) {
      routes.emplace_back(std::vector<routing::Phase>{MakePhase(0, edge_sequence[0], lane_s_range_tolerance_, {start},
                                                                {end}, road_network_)},
                          &road_network_);
      continue;
    }

    // Handles the case when edge_sequence has a length greater than 1. This implies the route has more than one
    // routing::Phase.
    std::vector<routing::Phase> phases;
    for (int i = 0; i < static_cast<int>(edge_sequence.size()); ++i) {
      const routing::graph::Edge& edge = edge_sequence[i];
      if (i == 0) {
        const std::optional<api::LaneEnd::Which> edge_end =
            routing::graph::DetermineEdgeEnd(graph_, edge, edge_sequence[i + 1]);
        MALIPUT_THROW_UNLESS(edge_end.has_value());
        phases.push_back(MakePhase(i, edge, lane_s_range_tolerance_, {start},
                                   ComputeAllRoadPositions(edge, edge_end.value(), edge_sequence[i + 1]),
                                   road_network_));
      } else if (i + 1 == static_cast<int>(edge_sequence.size())) {
        const std::optional<api::LaneEnd::Which> edge_end =
            routing::graph::DetermineEdgeEnd(graph_, edge, edge_sequence[i - 1]);
        MALIPUT_THROW_UNLESS(edge_end.has_value());
        phases.push_back(MakePhase(i, edge, lane_s_range_tolerance_,
                                   ComputeAllRoadPositions(edge, edge_end.value(), edge_sequence[i - 1]), {end},
                                   road_network_));
      } else {
        const std::optional<api::LaneEnd::Which> start_edge_end =
            routing::graph::DetermineEdgeEnd(graph_, edge, edge_sequence[i - 1]);
        MALIPUT_THROW_UNLESS(start_edge_end.has_value());
        const std::optional<api::LaneEnd::Which> finish_edge_end =
            routing::graph::DetermineEdgeEnd(graph_, edge, edge_sequence[i + 1]);
        MALIPUT_THROW_UNLESS(finish_edge_end.has_value());
        phases.push_back(MakePhase(i, edge, lane_s_range_tolerance_,
                                   ComputeAllRoadPositions(edge, start_edge_end.value(), edge_sequence[i - 1]),
                                   ComputeAllRoadPositions(edge, finish_edge_end.value(), edge_sequence[i + 1]),
                                   road_network_));
      }
    }
    routes.emplace_back(phases, &road_network_);
  }

  // Filter routes based on constraints.
  routes = FilterRoutes(routing_constraints, routes);

  // Validate end to end connectivity in resulting routes.
  ValidateEndToEndConnectivityInRoutes(routes);

  return routes;
}

}  // namespace maliput
