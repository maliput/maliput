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

#include <optional>
#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/routing/graph/graph.h"

namespace maliput {
namespace routing {
namespace graph {

/// Finds all Edge sequences in @p graph that join @p start with @p end.
///
/// @param graph The Graph to conduct the search.
/// @param start The start Node. It must exist in the @p graph.
/// @param end The end Node. It must exist in the @p graph.
/// @return A vector of vectors of Edges that joint @p start with @p end. When @p start and @p end are the same, the
/// returned vector is empty.
/// @throws maliput::common::assertion_error When any of @p start and @p end do not exist in @p graph.
std::vector<std::vector<Edge>> FindAllEdgeSequences(const Graph& graph, const Node& start, const Node& end);

/// Finds a Node in @p graph that is on the @p end extent of the Edge @p pos falls into.
///
/// @param graph The Graph to perform the search into.
/// @param pos The api::RoadPosition to match.
/// @param end The api::LaneEnd::Which indicating the side of the Edge.
/// @return An optional wrapping a Node when @p pos can be matched into an Edge.
/// @throws maliput::common::assertion_error When @p pos.lane is nullptr.
std::optional<Node> FindNode(const Graph& graph, const api::RoadPosition& pos, const api::LaneEnd::Which& end);

/// Determines the api::Lane::End::Which end that connects @p ref_end with @p target_end.
///
/// When there is no connection between @p ref_edge and @p target_edge, std::nullopt is returned.
/// Note: this method uses the @p graph connectivity, thus there might be no real connection between @p ref_edge
/// and @p target_end underlying api::Segments by means of api::BranchPoints.
/// When @p ref_edge.segment and @p target_edge.segment are equal, this function returns std::nullopt.
///
/// @param ref_edge The reference Edge.
/// @param target_edge The target Edge.
/// @param graph The graph to lookup Edges and Nodes.
/// @return An optional with the api::LaneEnd::Which end in @p ref_edge that connects with @p target_edge.
std::optional<api::LaneEnd::Which> DetermineEdgeEnd(const Edge& ref_edge, const Edge& target_edge, const Graph& graph);

}  // namespace graph
}  // namespace routing
}  // namespace maliput
