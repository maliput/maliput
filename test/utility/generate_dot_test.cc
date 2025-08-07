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
#include "maliput/common/maliput_error.h"
#include "maliput/routing/graph/graph.h"
#include "road_network_mocks.h"

namespace maliput {
namespace utility {
namespace test {
namespace {

using ::testing::Return;

using maliput::test::SegmentMock;

GTEST_TEST(GenerateDotStream, PassingNullOstreamThrows) {
  ASSERT_THROW({ GenerateDotStream(routing::graph::Graph{}, nullptr /* os */); }, common::assertion_error);
}

GTEST_TEST(GenerateDotStream, EmptyGraph) {
  const std::string kResult(R"(graph {
}
)");
  const routing::graph::Graph kGraph{};
  std::stringstream ss;

  GenerateDotStream(kGraph, &ss);

  ASSERT_EQ(kResult, ss.str());
}

/* This is the graph to be modeled:
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
GTEST_TEST(GenerateDotStream, PopulatedGraph) {
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
  const api::SegmentId kSegmentAId("S:A");
  const api::SegmentId kSegmentBId("S:B");
  const api::SegmentId kSegmentCId("S:C");
  const api::SegmentId kSegmentDId("S:D");
  EXPECT_CALL(segment_a, do_id()).WillRepeatedly(Return(kSegmentAId));
  EXPECT_CALL(segment_b, do_id()).WillRepeatedly(Return(kSegmentBId));
  EXPECT_CALL(segment_c, do_id()).WillRepeatedly(Return(kSegmentCId));
  EXPECT_CALL(segment_d, do_id()).WillRepeatedly(Return(kSegmentDId));
  const routing::graph::EdgeId kEdgeIdA(kSegmentAId.string());
  const routing::graph::EdgeId kEdgeIdB(kSegmentBId.string());
  const routing::graph::EdgeId kEdgeIdC(kSegmentCId.string());
  const routing::graph::EdgeId kEdgeIdD(kSegmentDId.string());
  const routing::graph::NodeId kNodeIdA{std::to_string(0u)};
  const routing::graph::NodeId kNodeIdB{std::to_string(1u)};
  const routing::graph::NodeId kNodeIdC{std::to_string(2u)};
  const routing::graph::NodeId kNodeIdD{std::to_string(3u)};
  const api::BranchPoint* kBranchPointA{reinterpret_cast<const api::BranchPoint*>(0x00010001)};
  const api::BranchPoint* kBranchPointB{reinterpret_cast<const api::BranchPoint*>(0x00010002)};
  const api::BranchPoint* kBranchPointC{reinterpret_cast<const api::BranchPoint*>(0x00010003)};
  const api::BranchPoint* kBranchPointD{reinterpret_cast<const api::BranchPoint*>(0x00010004)};
  const routing::graph::Edge kEdgeA{kEdgeIdA, &segment_a, kNodeIdA, kNodeIdB};
  const routing::graph::Edge kEdgeB{kEdgeIdB, &segment_b, kNodeIdB, kNodeIdC};
  const routing::graph::Edge kEdgeC{kEdgeIdC, &segment_c, kNodeIdB, kNodeIdC};
  const routing::graph::Edge kEdgeD{kEdgeIdD, &segment_d, kNodeIdC, kNodeIdD};
  const routing::graph::Node kNodeA{kNodeIdA, {kBranchPointA}, {kEdgeIdA}};
  const routing::graph::Node kNodeB{kNodeIdB, {kBranchPointB}, {kEdgeIdA, kEdgeIdB, kEdgeIdC}};
  const routing::graph::Node kNodeC{kNodeIdC, {kBranchPointC}, {kEdgeIdB, kEdgeIdC, kEdgeIdD}};
  const routing::graph::Node kNodeD{kNodeIdD, {kBranchPointD}, {kEdgeIdD}};
  const routing::graph::Graph kGraph{
      std::unordered_map<routing::graph::EdgeId, routing::graph::Edge>{
          {kEdgeIdA, kEdgeA}, {kEdgeIdB, kEdgeB}, {kEdgeIdC, kEdgeC}, {kEdgeIdD, kEdgeD}},
      std::unordered_map<routing::graph::NodeId, routing::graph::Node>{
          {kNodeIdA, kNodeA}, {kNodeIdB, kNodeB}, {kNodeIdC, kNodeC}, {kNodeIdD, kNodeD}}};
  std::stringstream ss;

  GenerateDotStream(kGraph, &ss);

  ASSERT_EQ(kResult, ss.str());
}

}  // namespace
}  // namespace test
}  // namespace utility
}  // namespace maliput
