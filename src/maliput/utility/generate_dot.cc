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
#include "maliput/utility/generate_dot.h"

#include <algorithm>
#include <unordered_set>

#include "maliput/api/segment.h"

namespace maliput {
namespace utility {

void GenerateDotStream(const routing::graph::Graph& graph, std::ostream* os) {
  MALIPUT_THROW_UNLESS(os != nullptr);

  (*os) << "graph {" << std::endl;
  for (const auto& id_edge : graph.edges) {
    (*os) << id_edge.second.node_a << " -- " << id_edge.second.node_b << " [ label = \""
          << id_edge.second.segment->id().string() << "\" ];" << std::endl;
  }
  (*os) << "}" << std::endl;
  os->flush();
}

void GenerateDotStream(const routing::graph::Graph& graph, const routing::Route& route, std::ostream* os) {
  MALIPUT_THROW_UNLESS(os != nullptr);

  std::unordered_set<const api::Segment*> segments;
  for (int i = 0; i < route.size(); ++i) {
    const api::Segment* segment = route.Get(i).start_positions().front().lane->segment();
    auto edge_it = std::find_if(graph.edges.begin(), graph.edges.end(), [segment](const auto& id_edge) {
      return id_edge.second.segment == segment;
    });
    MALIPUT_THROW_UNLESS(edge_it != graph.edges.end());
    segments.insert(segment);
  }

  (*os) << "graph {" << std::endl;
  for (const auto& id_edge : graph.edges) {
    (*os) << id_edge.second.node_a << " -- " << id_edge.second.node_b << " [ label = \""
          << id_edge.second.segment->id().string() << "\"";
    // Conditionally adds the red color to the api::Segments that correspond with a routing::Phase.
    if (segments.find(id_edge.second.segment) != segments.end()) {
      (*os) << " color = \"red\"";
    }
    (*os) << " ];" << std::endl;
  }
  (*os) << "}" << std::endl;
  os->flush();
}

}  // namespace utility
}  // namespace maliput
