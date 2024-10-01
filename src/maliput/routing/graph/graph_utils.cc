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
#include "maliput/routing/graph/graph_utils.h"

#include <algorithm>
#include <set>

#include "maliput/api/branch_point.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace routing {
namespace graph {
namespace {

// Recursively searches @p graph for a sequence of edges that go from @p start to @p end.
std::vector<std::vector<Edge>> FindAllEdgeSequencesHelper(const Graph& graph, const Node& start, const Node& end,
                                                          const std::set<NodeId>& visited_nodes) {
  std::vector<std::vector<Edge>> result;
  const std::set<EdgeId>& edges_to_traverse = start.edges;
  for (const EdgeId& edge_id : edges_to_traverse) {
    const Edge& edge = graph.edges.at(edge_id);
    const Node& node_to_visit = graph.nodes.at(edge.node_a == start.id ? edge.node_b : edge.node_a);
    // Check for visited nodes.
    if (visited_nodes.find(node_to_visit.id) != visited_nodes.end()) {
      continue;
    }
    // Check for matching end node.
    if (node_to_visit.id == end.id) {
      result.push_back({edge});
    } else {  // Find sequences of edges leading to the end.
      std::set<NodeId> new_visited_nodes = visited_nodes;
      new_visited_nodes.insert(node_to_visit.id);
      const auto subsequences = FindAllEdgeSequencesHelper(graph, node_to_visit, end, new_visited_nodes);
      for (std::vector<Edge> subsequence : subsequences) {
        subsequence.insert(subsequence.begin(), edge);
        result.push_back(subsequence);
      }
    }
  }
  return result;
}

// Convenient struct to hold both incoming and ongoing LaneEnds.
// It will be used in HasUTurn to identify when a sequence of Edges contains a U-turn.
struct ConnectionSet {
  api::LaneEnd::Which incoming{api::LaneEnd::Which::kStart};
  api::LaneEnd::Which ongoing{api::LaneEnd::Which::kStart};
};

// Builds a ConnectionSet from two adjacent Edges.
//
// The method will look for the common node of @p incoming_edge and @p ongoing_edge.
// Once identified, using the node_a as the LaneEnd::Which::kStart and node_b as the
// LaneEnd::Which::kFinish, it'll assign a result.
// When edges are not adjacent, this function throws.
//
// @param incoming_edge The incoming Edge. Its segment must not be nullptr.
// @param ongoing_edge The ongoing Edge. Its segment must not be nullptr.
// @return A ConnectionSet.
// @throws common::assertion_error When @p incoming_edge's or @p ongoing_edge's segment is nullptr.
// @throws common::assertion_error When @p incoming_edge and @p ongoing_edge are not adjacent.
ConnectionSet BuildConnectionSet(const Edge& incoming_edge, const Edge& ongoing_edge) {
  MALIPUT_THROW_UNLESS(incoming_edge.segment != nullptr);
  MALIPUT_THROW_UNLESS(ongoing_edge.segment != nullptr);
  ConnectionSet result{};
  if (incoming_edge.node_a == ongoing_edge.node_a) {
    result.incoming = api::LaneEnd::Which::kStart;
    result.ongoing = api::LaneEnd::Which::kStart;
  } else if (incoming_edge.node_a == ongoing_edge.node_b) {
    result.incoming = api::LaneEnd::Which::kStart;
    result.ongoing = api::LaneEnd::Which::kFinish;
  } else if (incoming_edge.node_b == ongoing_edge.node_a) {
    result.incoming = api::LaneEnd::Which::kFinish;
    result.ongoing = api::LaneEnd::Which::kStart;
  } else if (incoming_edge.node_b == ongoing_edge.node_b) {
    result.incoming = api::LaneEnd::Which::kFinish;
    result.ongoing = api::LaneEnd::Which::kFinish;
  } else {
    // It was impossible to find a connection between the two edges, an error has occurred somewhere else.
    MALIPUT_THROW_MESSAGE("maliput::routing::graph::BuildConnectionSet(): code must not reach here.");
  }
  return result;
}

// Evaluates whether @p ref_edge and @p target_edge connect in at least one common api::BranchPoint
// and from different sides of the api::BranchPoint.
//
// Computes the ConnectionSet for @p ref_edge and @p target_edge and then the set of api::BranchPoints at the
// corresponding sides of @p ref_edge and @p target_edge. The intersection of the two sets yields the common
// api::BranchPoints. When the resulting set is empty, this function returns false.
// From the common api::BranchPoints, one is taken and the api::LaneEndSets are evaluated to identify @p ref_edge
// and @p target_edge on each side. Finally, the matched sides are compared and this method returns true when
// they are different.
//
// @param ref_edge The incoming Edge.
// @param target_edge The ongoing Edge.
// @return true When @p ref_edge connects with @p target_edge from different sides to at least one api::BranchPoint.
bool EdgesConnectToBranchPointsFromOpposingSides(const Edge& ref_edge, const Edge& target_edge) {
  const ConnectionSet connection_set = BuildConnectionSet(ref_edge, target_edge);
  auto compute_connecting_edges = [](const Edge& edge, api::LaneEnd::Which end) {
    std::set<const api::BranchPoint*> result;
    for (int i = 0; i < edge.segment->num_lanes(); ++i) {
      result.insert(edge.segment->lane(i)->GetBranchPoint(end));
    }
    return result;
  };
  const std::set<const api::BranchPoint*> ref_branchpoints =
      compute_connecting_edges(ref_edge, connection_set.incoming);
  const std::set<const api::BranchPoint*> target_branchpoints =
      compute_connecting_edges(target_edge, connection_set.ongoing);
  std::set<const api::BranchPoint*> common_bps;
  std::copy_if(ref_branchpoints.begin(), ref_branchpoints.end(), std::inserter(common_bps, common_bps.end()),
               [bps = target_branchpoints](const api::BranchPoint* bp) { return bps.find(bp) != bps.end(); });
  if (common_bps.empty()) {
    return false;
  }
  auto is_edge_on_a_side = [bp = *(common_bps.begin())](const Edge& edge) {
    const api::LaneEndSet* a_side = bp->GetASide();
    for (int le = 0; le < a_side->size(); ++le) {
      const api::LaneEnd lane_end = a_side->get(le);
      if (lane_end.lane->segment() == edge.segment) {
        return true;
      }
    }
    return false;
  };
  return is_edge_on_a_side(ref_edge) != is_edge_on_a_side(target_edge);
}

// Whether a sequence of Edges are drivable.
//
// Sequences of Edges are drivable when evaluating consecutive pairs across the sequence, the underlying
// api::Segments of the Edges share at least one api::BranchPoint and the api::Lanes of each Edge connect
// in the api::BranchPoint on opposing sides of the api::BranchPoints.
//
// @param edge_sequence A sequence of Edges.
// @return true When the sequence is drivable.
// @throws common::assertion_error When @p edge_sequence is empty.
bool IsSequenceDrivable(const std::vector<Edge>& edge_sequence) {
  // Analyze preconditions.
  MALIPUT_THROW_UNLESS(!edge_sequence.empty());
  // Escape condition: when edge_sequence has one edge, it is always drivable.
  if (edge_sequence.size() == 1u) {
    return true;
  }
  for (size_t i = 0u; i < edge_sequence.size() - 1u; ++i) {
    if (!EdgesConnectToBranchPointsFromOpposingSides(edge_sequence[i], edge_sequence[i + 1])) {
      return false;
    }
  }
  return true;
}

}  // namespace

std::vector<std::vector<Edge>> FindAllEdgeSequences(const Graph& graph, const Node& start, const Node& end) {
  MALIPUT_THROW_UNLESS(graph.nodes.find(start.id) != graph.nodes.end());
  MALIPUT_THROW_UNLESS(graph.nodes.find(end.id) != graph.nodes.end());

  if (start.id == end.id) {
    return {};
  }
  std::vector<std::vector<Edge>> unfiltered_result = FindAllEdgeSequencesHelper(graph, start, end, {start.id});
  unfiltered_result.erase(
      std::remove_if(unfiltered_result.begin(), unfiltered_result.end(), std::not_fn(IsSequenceDrivable)),
      unfiltered_result.end());
  return unfiltered_result;
}

std::optional<Node> FindNode(const Graph& graph, const api::Lane& lane, const api::LaneEnd::Which& end) {
  const api::BranchPoint* branch_point = lane.GetBranchPoint(end);
  for (const auto& id_node : graph.nodes) {
    const Node& node = id_node.second;
    if (node.branch_points.find(branch_point) != node.branch_points.end()) {
      return {node};
    }
  }
  return {};
}

std::optional<api::LaneEnd::Which> DetermineEdgeEnd(const Graph& graph, const Edge& ref_edge, const Edge& target_edge) {
  if (ref_edge.segment == target_edge.segment) {
    return std::nullopt;
  }
  const Node& node_a = graph.nodes.at(ref_edge.node_a);
  if (node_a.edges.find(target_edge.id) != node_a.edges.end()) {
    return api::LaneEnd::Which::kStart;
  }
  const Node& node_b = graph.nodes.at(ref_edge.node_b);
  if (node_b.edges.find(target_edge.id) != node_b.edges.end()) {
    return api::LaneEnd::Which::kFinish;
  }
  return std::nullopt;
}

}  // namespace graph
}  // namespace routing
}  // namespace maliput
