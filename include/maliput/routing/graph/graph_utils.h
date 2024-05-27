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

#include "maliput/api/lane.h"
#include "maliput/api/segment.h"
#include "maliput/routing/graph/graph.h"

namespace maliput {
namespace routing {
namespace graph {

/// @brief Convenient struct to group those api::Lanes within an api::Segment.
/// @details This data type offers a filtered view of Edge, which becomes useful when
/// combining in a sequence various Edges to join Node A with Node B.
struct SubSegment {
  /// @brief The api::Segment that refers to.
  const api::Segment* segment{};
  /// @brief The list of api::Lanes within `segment` that are selected.
  std::vector<const api::Lane*> lanes;
};

/// @brief Finds all Edge sequences in @p graph that join @p start with @p end.
/// @param graph The Graph to conduct the search.
/// @param start The start Node. It must exist in the @p graph.
/// @param end The end Node. It must exist in the @p graph.
/// @return A vector of vectors of Edges that joint @p start with @p end. When @p start and @p end are the same, the
/// returned vector is empty.
/// @throws maliput::common::assertion_error When any of @p start and @p end do not exist in @p graph.
std::vector<std::vector<Edge>> FindAllEdgeSequences(const Graph& graph, const Node& start, const Node& end);

/// @brief Filters api::Lanes from Edges in @p edge_sequences to leave out only api::Lanes in SubSegments that allow the
/// connectivity of the succeeding api::Segment.
/// @param edge_sequences The result of FindAllEdgeSequences().
/// @return A vector of vectors of SubSegments which contain the list of api::Segment and filtered api::Lanes in them.
/// @throws maliput::common::assertion_error When any element in @p edge_sequences is an empty sequence.
std::vector<std::vector<SubSegment>> FilterLanesFromEdges(const std::vector<std::vector<Edge>>& edge_sequences);

/// @brief Finds a Node in @p graph that is on the @p end extent of the Edge @p pos falls into.
/// @param graph The Graph to perform the search into.
/// @param pos The api::RoadPosition to match.
/// @param end The api::LaneEnd::Which indicating the side of the Edge.
/// @return An optional wrapping a Node when @p pos can be matched into an Edge.
/// @throws maliput::common::assertion_error When @p pos.lane is nullptr.
std::optional<Node> FindNode(const Graph& graph, const api::RoadPosition& pos, const api::LaneEnd::Which& end);

}  // namespace graph
}  // namespace routing
}  // namespace maliput
