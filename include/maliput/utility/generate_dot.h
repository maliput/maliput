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

#include <ostream>

#include "maliput/routing/graph/graph.h"
#include "maliput/routing/route.h"

namespace maliput {
namespace utility {

/// Generates a string representation in @p os of the @p graph using the DOT language.
///
/// routing::graph::Edges are created as bidirectional, and labels are added to indicate api::Segment::id().
/// routing::graph::Nodes only indicate their routing::graph::NodeId.
///
/// @param graph The routing::graph::Graph to serialize.
/// @param os A pointer to a std::ostream to serialize the @p graph. It must not be nullptr.
/// @throws common::assertion_error When @p os is nullptr.
void GenerateDotStream(const routing::graph::Graph& graph, std::ostream* os);

/// Generates a string representation in @p os of the @p graph and @p route using the DOT language.
///
/// routing::graph::Edges are created as bidirectional, and labels are added to indicate api::Segment::id().
/// routing::graph::Nodes only indicate their routing::graph::NodeId. The @p route is indicated by changing the color
/// to red of edges where it lays.
///
/// @param graph The routing::graph::Graph to serialize.
/// @param route The routing::Route to highlight in red on top of the @p graph.
/// @param os A pointer to a std::ostream to serialize the @p graph. It must not be nullptr.
/// @throws common::assertion_error When @p os is nullptr.
/// @throws common::assertion_error When @p route's involved api::Segments are not in @p graph.
void GenerateDotStream(const routing::graph::Graph& graph, const routing::Route& route, std::ostream* os);


}  // namespace utility
}  // namespace maliput
