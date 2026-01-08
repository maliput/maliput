// BSD 3-Clause License
//
// Copyright (c) 2024-2026, Woven by Toyota. All rights reserved.
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

#include <set>
#include <unordered_map>

#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
#include "maliput/api/type_specific_identifier.h"

namespace maliput {
namespace routing {
namespace graph {

/// Persistent identifier for an Edge in a Graph. It is guaranteed that
/// the aliased type will remain the same.
using EdgeId = api::TypeSpecificIdentifier<struct Edge>;

/// Persistent identifier for a Node in a Graph. It is guaranteed that
/// the aliased type will remain the same.
using NodeId = api::TypeSpecificIdentifier<struct Node>;

/// An edge wraps a pointer to an api::Segment and the identities of the
/// connecting nodes. An api::Segment instead of a api::Lane wrapped to allow Routers to
/// further refine which which specific api::Lanes to include in a final Route.
struct Edge {
  /// @brief The ID of the Edge.
  EdgeId id;
  /// @brief The api::Segment this Edge maps to.
  const api::Segment* segment{};
  /// @brief The NodeId at one extent of the Edge.
  NodeId node_a;
  /// @brief The NodeId at the other extent of the Edge.
  NodeId node_b;
};

/// A node wraps a bundle of api::BranchPoints and Edges. The api::BranchPoints are at
/// the ends of the api::Lanes in the Edges of this node, and connect the incoming
/// and outgoing Edges from this node.
struct Node {
  /// @brief The ID of the Node.
  NodeId id;
  /// @brief The set of api::BranchPoints that is contained within this Node.
  std::set<const api::BranchPoint*> branch_points;
  /// @brief The set of incoming and outgoing Edges to the Node.
  std::set<EdgeId> edges;
};

/// Basic type to hold the graph structure.
struct Graph {
  /// @brief The collection of Edges in this Graph.
  std::unordered_map<EdgeId, Edge> edges;
  /// @brief The collection of Nodes in this Graph.
  std::unordered_map<NodeId, Node> nodes;
};

/// Builds a Graph from @p rg.
///
/// Matches the api::Segments in @p rg to each Node, and bundles api::BranchPoints
/// at the extents of an api::Segment whose api::Lanes on one side connect to another set of
/// api::Lanes belonging to another api::Segment.
///
/// @param rg api::RoadGeometry to build a Graph from. It must not be nullptr.
/// @return A Graph.
/// @throws maliput::common::assertion_error When @p rg is nullptr.
Graph BuildGraph(const api::RoadGeometry* rg);

}  // namespace graph
}  // namespace routing
}  // namespace maliput
