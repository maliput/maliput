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
#include "maliput/routing/graph/graph.h"

#include <optional>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace routing {
namespace graph {
namespace {

// @brief Gets the set of api::BranchPoints for a given @p segment using all at @p end of the api::Lanes in @p segment.
// @param segment The api::Segment whose api::Lanes will be queried to retrieve the api::BranchPoints.
// @param end The side of api::Lanes in @p segment.
// @return A set of api::BranchPoints.
std::set<const api::BranchPoint*> GetBranchPoints(const api::Segment* segment, const api::LaneEnd::Which end) {
  MALIPUT_THROW_UNLESS(segment != nullptr);
  std::set<const api::BranchPoint*> branch_points;
  for (int l_id = 0; l_id < segment->num_lanes(); ++l_id) {
    const api::Lane* lane = segment->lane(l_id);
    branch_points.insert(lane->GetBranchPoint(end));
  }
  MALIPUT_THROW_UNLESS(branch_points.size() == static_cast<size_t>(segment->num_lanes()));
  return branch_points;
}

// @brief Finds a Node in @p graph by containing @p branch_points.
// @param graph The Graph containing the Nodes to look for.
// @param branch_points The set of api::BranchPoints whose items should be found in one of the Nodes in @p graph.
// @return An optional containing the matched Node. Otherwise, std::nullopt.
std::optional<Node> FindNode(const Graph& graph, std::set<const api::BranchPoint*> branch_points) {
  for (const auto& id_node : graph.nodes) {
    if (std::any_of(branch_points.begin(), branch_points.end(), [n = id_node.second](const auto bp) {
          return n.branch_points.find(bp) != n.branch_points.end();
        })) {
      return {id_node.second};
    }
  }
  return {};
}

/// @brief Gets a Node from @p graph, or creates one, with @p branch_points.
/// @details When there is no Node found in @p graph, a new Node is created and returned. Note that
/// @p graph is not changed, but the returned is, it'll always contain at least @p branch_points.
/// @param graph The Graph to look for the matching Node.
/// @param branch_points The set of api::BranchPoints to match the Node in @p graph.
/// @return A Node whose api::BranchPoints set contains @p branch_points.
Node GetNode(const Graph& graph, std::set<const api::BranchPoint*> branch_points) {
  // TODO(agalbachicar): Consider making something better for this.
  static size_t node_id = 0u;

  const std::optional<Node> node_result = FindNode(graph, branch_points);
  if (node_result.has_value()) {
    Node node = node_result.value();
    for (const api::BranchPoint* bp : branch_points) {
      node.branch_points.insert(bp);
    }
    return node_result.value();
  }
  return Node{node_id++, branch_points, {}};
}

}  // namespace

Graph BuildGraph(const api::RoadGeometry* rg) {
  MALIPUT_THROW_UNLESS(rg != nullptr);

  Graph graph;

  for (int j_id = 0; j_id < rg->num_junctions(); ++j_id) {
    const api::Junction* junction = rg->junction(j_id);
    for (int s_id = 0; s_id < junction->num_segments(); ++s_id) {
      const api::Segment* segment = junction->segment(s_id);

      const std::set<const api::BranchPoint*> start_branch_points =
          GetBranchPoints(segment, api::LaneEnd::Which::kStart);
      const std::set<const api::BranchPoint*> end_branch_points =
          GetBranchPoints(segment, api::LaneEnd::Which::kFinish);

      Node start_node = GetNode(graph, start_branch_points);
      Node end_node = GetNode(graph, end_branch_points);

      Edge edge{segment, start_node.id, end_node.id};

      start_node.edges.insert(segment);
      end_node.edges.insert(segment);

      graph.edges[segment] = edge;
      graph.nodes[start_node.id] = start_node;
      graph.nodes[end_node.id] = end_node;
    }
  }

  return graph;
}

}  // namespace graph
}  // namespace routing
}  // namespace maliput
