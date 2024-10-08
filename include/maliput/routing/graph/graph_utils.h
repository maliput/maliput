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
/// @param graph The Graph in which to conduct the search.
/// @param start The start Node. It must exist in the @p graph.
/// @param end The end Node. It must exist in the @p graph.
/// @return A vector of vectors of Edges that join @p start with @p end. When @p start and @p end are the same, the
/// returned vector is empty.
/// @throws maliput::common::assertion_error When any of @p start and @p end do not exist in @p graph.
std::vector<std::vector<Edge>> FindAllEdgeSequences(const Graph& graph, const Node& start, const Node& end);

/// Finds a Node in @p graph that contains the api::BranchPoint at the @p end extent of the @p lane.
///
/// @param graph The Graph in which to conduct the search.
/// @param lane The api::Lane whose @p end api::BranchPoint should be in @p graph.
/// @param end The api::LaneEnd::Which indicating the side of the Edge.
/// @return An optional wrapping a Node when @p pos can be matched into an Edge.
std::optional<Node> FindNode(const Graph& graph, const api::Lane& lane, const api::LaneEnd::Which& end);

/// Determines the api::Lane::End::Which end of an api::Lane in @p ref_edge that connects to an api::Lane in
/// @p target_edge.
///
/// When there is no direct connection between @p ref_edge and @p target_edge, std::nullopt is returned.
/// Note: this method uses the @p graph connectivity, thus there might be no real connection between @p ref_edge
/// and @p target_end underlying api::Segments by means of api::BranchPoints, thus returning std::nullopt.
/// When @p ref_edge.segment and @p target_edge.segment are equal, this function returns std::nullopt.
///
/// @param graph The Graph containing @p ref_edge and @p target_edge.
/// @param ref_edge The reference Edge.
/// @param target_edge The target Edge.
/// @return An optional with the api::LaneEnd::Which end of an api::Lane in @p ref_edge that connects with another
/// api::Lane in @p target_edge.
std::optional<api::LaneEnd::Which> DetermineEdgeEnd(const Graph& graph, const Edge& ref_edge, const Edge& target_edge);

}  // namespace graph
}  // namespace routing
}  // namespace maliput
