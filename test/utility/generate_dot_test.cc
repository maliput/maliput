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

#include <sstream>
#include <string>
#include <unordered_map>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/branch_point.h"
#include "maliput/api/segment.h"
#include "maliput/common/assertion_error.h"
#include "maliput/routing/graph/graph.h"
#include "road_network_mocks.h"

namespace maliput {
namespace utility {
namespace test {
namespace {

using ::testing::Return;

using maliput::test::SegmentMock;

GTEST_TEST(GenerateDotFile, PassingNullOstreamThrows) {
  ASSERT_THROW({ GenerateDotFile(routing::graph::Graph{}, nullptr /* os */); }, common::assertion_error);
}

GTEST_TEST(GenerateDotFile, EmptyGraph) {
  const std::string kResult(R"(graph {
}
)");
  const routing::graph::Graph kGraph{};
  std::stringstream ss;

  GenerateDotFile(kGraph, &ss);

  ASSERT_EQ(kResult, ss.str());
}

/* This is the graph to be modelled:
 *
 * <pre>
 *            ----
 *           /    \  > S:B
 *          /      \
 * x-------x  B     x-------x
 * A ^      \      / C ^    D
 *   S:A     \    /    S:D
 *            ---- > S:C
 * </pre>
 */
GTEST_TEST(GenerateDotFile, PopulatedGraph) {
  // The order of the items in the result is a product of the order of initialization.
  // Review it when changing the initialization.
  const std::string kResult(R"(graph {
2 -- 3 [ label = "S:D" ];
1 -- 2 [ label = "S:C" ];
1 -- 2 [ label = "S:B" ];
0 -- 1 [ label = "S:A" ];
}
)");
  SegmentMock segment_a;
  SegmentMock segment_b;
  SegmentMock segment_c;
  SegmentMock segment_d;
  const api::Segment* kSegmentA{&segment_a};
  const api::Segment* kSegmentB{&segment_b};
  const api::Segment* kSegmentC{&segment_c};
  const api::Segment* kSegmentD{&segment_d};
  const api::SegmentId kSegmentAId("S:A");
  const api::SegmentId kSegmentBId("S:B");
  const api::SegmentId kSegmentCId("S:C");
  const api::SegmentId kSegmentDId("S:D");
  EXPECT_CALL(segment_a, do_id()).WillRepeatedly(Return(kSegmentAId));
  EXPECT_CALL(segment_b, do_id()).WillRepeatedly(Return(kSegmentBId));
  EXPECT_CALL(segment_c, do_id()).WillRepeatedly(Return(kSegmentCId));
  EXPECT_CALL(segment_d, do_id()).WillRepeatedly(Return(kSegmentDId));
  const routing::graph::NodeId kNodeIdA{0u};
  const routing::graph::NodeId kNodeIdB{1u};
  const routing::graph::NodeId kNodeIdC{2u};
  const routing::graph::NodeId kNodeIdD{3u};
  const api::BranchPoint* kBranchPointA{reinterpret_cast<const api::BranchPoint*>(0x00010001)};
  const api::BranchPoint* kBranchPointB{reinterpret_cast<const api::BranchPoint*>(0x00010002)};
  const api::BranchPoint* kBranchPointC{reinterpret_cast<const api::BranchPoint*>(0x00010003)};
  const api::BranchPoint* kBranchPointD{reinterpret_cast<const api::BranchPoint*>(0x00010004)};
  const routing::graph::Edge kEdgeA{kSegmentA, kNodeIdA, kNodeIdB};
  const routing::graph::Edge kEdgeB{kSegmentB, kNodeIdB, kNodeIdC};
  const routing::graph::Edge kEdgeC{kSegmentC, kNodeIdB, kNodeIdC};
  const routing::graph::Edge kEdgeD{kSegmentD, kNodeIdC, kNodeIdD};
  const routing::graph::Node kNodeA{kNodeIdA, {kBranchPointA}, {kSegmentA}};
  const routing::graph::Node kNodeB{kNodeIdB, {kBranchPointB}, {kSegmentA, kSegmentB, kSegmentC}};
  const routing::graph::Node kNodeC{kNodeIdC, {kBranchPointC}, {kSegmentB, kSegmentC, kSegmentD}};
  const routing::graph::Node kNodeD{kNodeIdD, {kBranchPointD}, {kSegmentD}};
  const routing::graph::Graph kGraph{
      std::unordered_map<routing::graph::EdgeId, routing::graph::Edge>{
          {kSegmentA, kEdgeA}, {kSegmentB, kEdgeB}, {kSegmentC, kEdgeC}, {kSegmentD, kEdgeD}},
      std::unordered_map<routing::graph::NodeId, routing::graph::Node>{
          {kNodeIdA, kNodeA}, {kNodeIdB, kNodeB}, {kNodeIdC, kNodeC}, {kNodeIdD, kNodeD}}};
  std::stringstream ss;

  GenerateDotFile(kGraph, &ss);

  ASSERT_EQ(kResult, ss.str());
}

}  // namespace
}  // namespace test
}  // namespace utility
}  // namespace maliput
