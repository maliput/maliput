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

#include <set>

#include "maliput/api/branch_point.h"
#include "maliput/api/lane_data.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace routing {
namespace graph {
namespace {

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

}  // namespace

std::vector<std::vector<Edge>> FindAllEdgeSequences(const Graph& graph, const Node& start, const Node& end) {
  MALIPUT_THROW_UNLESS(graph.nodes.find(start.id) != graph.nodes.end());
  MALIPUT_THROW_UNLESS(graph.nodes.find(end.id) != graph.nodes.end());

  if (start.id == end.id) {
    return {};
  }
  return FindAllEdgeSequencesHelper(graph, start, end, {start.id});
}

std::optional<Node> FindNode(const Graph& graph, const api::RoadPosition& pos, const api::LaneEnd::Which& end) {
  MALIPUT_THROW_UNLESS(pos.lane != nullptr);
  const api::BranchPoint* branch_point = pos.lane->GetBranchPoint(end);
  for (const auto& id_node : graph.nodes) {
    const Node& node = id_node.second;
    if (node.branch_points.find(branch_point) != node.branch_points.end()) {
      return {node};
    }
  }
  return {};
}

}  // namespace graph
}  // namespace routing
}  // namespace maliput