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

#include <optional>
#include <set>
#include <unordered_map>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/branch_point.h"
#include "maliput/api/segment.h"
#include "maliput/common/assertion_error.h"
#include "maliput/routing/graph/graph.h"
#include "road_network_mocks.h"

namespace maliput {
namespace routing {
namespace graph {
namespace test {
namespace {

using ::testing::_;
using ::testing::Eq;
using ::testing::Return;

using maliput::test::BranchPointMock;
using maliput::test::JunctionMock;
using maliput::test::LaneMock;
using maliput::test::RoadGeometryMock;
using maliput::test::SegmentMock;

// Tests that using nodes not in the graph makes the function to throw.
GTEST_TEST(FindAllEdgeSequences, NodesOutsideTheGraphThrow) {
  const EdgeId kEdgeId("edge_id");
  const api::Segment* kSegment{reinterpret_cast<const api::Segment*>(0x00000001)};
  const NodeId kNodeIdA("0");
  const NodeId kNodeIdB("1");
  const NodeId kNodeIdC("2");
  const api::BranchPoint* kBranchPointA{reinterpret_cast<const api::BranchPoint*>(0x00010001)};
  const api::BranchPoint* kBranchPointB{reinterpret_cast<const api::BranchPoint*>(0x00010002)};
  const api::BranchPoint* kBranchPointC{reinterpret_cast<const api::BranchPoint*>(0x00010003)};
  const Edge kEdge{kEdgeId, kSegment, kNodeIdA, kNodeIdB};
  const Node kNodeA{kNodeIdA, {kBranchPointA}, {kEdgeId}};
  const Node kNodeB{kNodeIdB, {kBranchPointB}, {kEdgeId}};
  const Node kNodeC{kNodeIdC, {kBranchPointC}, {kEdgeId}};
  const Graph graph{std::unordered_map<EdgeId, Edge>{{kEdgeId, kEdge}},
                    std::unordered_map<NodeId, Node>{{kNodeIdA, kNodeA}, {kNodeIdB, kNodeB}}};

  ASSERT_THROW({ FindAllEdgeSequences(graph, kNodeC, kNodeB); }, common::assertion_error);
  ASSERT_THROW({ FindAllEdgeSequences(graph, kNodeA, kNodeC); }, common::assertion_error);
}

// Basic graph with two nodes and one edge joining them yields the following results:
// Querying:
// - A -> B: the only edge.
// - B -> A: the only edge.
// - A -> A: empty.
// - B -> B: empty.
GTEST_TEST(FindAllEdgeSequences, OneEdgeGraph) {
  const EdgeId kEdgeId("edge_id");
  const api::Segment* kSegment{reinterpret_cast<const api::Segment*>(0x00000001)};
  const NodeId kNodeIdA("0");
  const NodeId kNodeIdB("1");
  const api::BranchPoint* kBranchPointA{reinterpret_cast<const api::BranchPoint*>(0x00010001)};
  const api::BranchPoint* kBranchPointB{reinterpret_cast<const api::BranchPoint*>(0x00010002)};
  const Edge kEdge{kEdgeId, kSegment, kNodeIdA, kNodeIdB};
  const Node kNodeA{kNodeIdA, {kBranchPointA}, {kEdgeId}};
  const Node kNodeB{kNodeIdB, {kBranchPointB}, {kEdgeId}};
  const Graph graph{std::unordered_map<EdgeId, Edge>{{kEdgeId, kEdge}},
                    std::unordered_map<NodeId, Node>{{kNodeIdA, kNodeA}, {kNodeIdB, kNodeB}}};

  {
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeA, kNodeB);

    ASSERT_EQ(1u, edge_sequences.size());
    const std::vector<Edge>& edge_sequence = edge_sequences[0];
    ASSERT_EQ(1u, edge_sequence.size());
    ASSERT_EQ(kEdgeId, edge_sequence[0].id);
    ASSERT_EQ(kSegment, edge_sequence[0].segment);
    ASSERT_EQ(kNodeIdA, edge_sequence[0].node_a);
    ASSERT_EQ(kNodeIdB, edge_sequence[0].node_b);
  }
  {
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeB, kNodeA);

    ASSERT_EQ(1u, edge_sequences.size());
    const std::vector<Edge>& edge_sequence = edge_sequences[0];
    ASSERT_EQ(1u, edge_sequence.size());
    ASSERT_EQ(kEdgeId, edge_sequence[0].id);
    ASSERT_EQ(kSegment, edge_sequence[0].segment);
    ASSERT_EQ(kNodeIdA, edge_sequence[0].node_a);
    ASSERT_EQ(kNodeIdB, edge_sequence[0].node_b);
  }
  {
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeA, kNodeA);

    ASSERT_TRUE(edge_sequences.empty());
  }
  {
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeB, kNodeB);

    ASSERT_TRUE(edge_sequences.empty());
  }
}

// The graph shows the case of a roundabout or any other geometry where we have
// one branching node and another merging node with two paths to go from one to the
// other. One possible geometry follows:
//
// <pre>
//    ----
//   /    \  > kSegmentA
//  /      \
// x  A     x B
//  \      /
//   \    /  > kSegmentB
//    ----
// </pre>
GTEST_TEST(FindAllEdgeSequences, Roundabout) {
  const EdgeId kEdgeIdA("edge_a");
  const EdgeId kEdgeIdB("edge_b");
  const api::Segment* kSegmentA{reinterpret_cast<const api::Segment*>(0x00000001)};
  const api::Segment* kSegmentB{reinterpret_cast<const api::Segment*>(0x00000002)};
  const NodeId kNodeIdA("0");
  const NodeId kNodeIdB("1");
  const api::BranchPoint* kBranchPointA{reinterpret_cast<const api::BranchPoint*>(0x00010001)};
  const api::BranchPoint* kBranchPointB{reinterpret_cast<const api::BranchPoint*>(0x00010002)};
  const Edge kEdgeA{kEdgeIdA, kSegmentA, kNodeIdA, kNodeIdB};
  const Edge kEdgeB{kEdgeIdB, kSegmentB, kNodeIdB, kNodeIdA};
  const Node kNodeA{kNodeIdA, {kBranchPointA}, {kEdgeIdA, kEdgeIdB}};
  const Node kNodeB{kNodeIdB, {kBranchPointB}, {kEdgeIdA, kEdgeIdB}};
  const Graph graph{std::unordered_map<EdgeId, Edge>{{kEdgeIdA, kEdgeA}, {kEdgeIdB, kEdgeB}},
                    std::unordered_map<NodeId, Node>{{kNodeIdA, kNodeA}, {kNodeIdB, kNodeB}}};

  {
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeA, kNodeB);

    ASSERT_EQ(2u, edge_sequences.size());
    const std::vector<Edge>& edge_sequence_1 = edge_sequences[0];
    const std::vector<Edge>& edge_sequence_2 = edge_sequences[1];
    ASSERT_EQ(1u, edge_sequence_1.size());
    ASSERT_EQ(1u, edge_sequence_2.size());
    ASSERT_NE(edge_sequence_1[0].id, edge_sequence_2[0].id);
    ASSERT_TRUE(edge_sequence_1[0].id == kEdgeIdA || edge_sequence_1[0].id == kEdgeIdB);
    ASSERT_TRUE(edge_sequence_2[0].id == kEdgeIdA || edge_sequence_2[0].id == kEdgeIdB);
  }
}

// Extends the case of the graph to include one entry and one exit to the diverging-merging path:
//
// <pre>
//            ----
//           /    \  > S:B
//          /      \
// x-------x  B     x-------x
// A ^      \      / C ^    D
//   S:A     \    /    S:D
//            ---- > S:C
// </pre>
GTEST_TEST(FindAllEdgeSequences, RoundaboutWithEntryAndExit) {
  const EdgeId kEdgeIdA("edge_a");
  const EdgeId kEdgeIdB("edge_b");
  const EdgeId kEdgeIdC("edge_c");
  const EdgeId kEdgeIdD("edge_d");
  const api::Segment* kSegmentA{reinterpret_cast<const api::Segment*>(0x00000001)};
  const api::Segment* kSegmentB{reinterpret_cast<const api::Segment*>(0x00000002)};
  const api::Segment* kSegmentC{reinterpret_cast<const api::Segment*>(0x00000003)};
  const api::Segment* kSegmentD{reinterpret_cast<const api::Segment*>(0x00000004)};
  const NodeId kNodeIdA("0");
  const NodeId kNodeIdB("1");
  const NodeId kNodeIdC("2");
  const NodeId kNodeIdD("3");
  const api::BranchPoint* kBranchPointA{reinterpret_cast<const api::BranchPoint*>(0x00010001)};
  const api::BranchPoint* kBranchPointB{reinterpret_cast<const api::BranchPoint*>(0x00010002)};
  const api::BranchPoint* kBranchPointC{reinterpret_cast<const api::BranchPoint*>(0x00010003)};
  const api::BranchPoint* kBranchPointD{reinterpret_cast<const api::BranchPoint*>(0x00010004)};
  const Edge kEdgeA{kEdgeIdA, kSegmentA, kNodeIdA, kNodeIdB};
  const Edge kEdgeB{kEdgeIdB, kSegmentB, kNodeIdB, kNodeIdC};
  const Edge kEdgeC{kEdgeIdC, kSegmentC, kNodeIdB, kNodeIdC};
  const Edge kEdgeD{kEdgeIdD, kSegmentD, kNodeIdC, kNodeIdD};
  const Node kNodeA{kNodeIdA, {kBranchPointA}, {kEdgeIdA}};
  const Node kNodeB{kNodeIdB, {kBranchPointB}, {kEdgeIdA, kEdgeIdB, kEdgeIdC}};
  const Node kNodeC{kNodeIdC, {kBranchPointC}, {kEdgeIdB, kEdgeIdC, kEdgeIdD}};
  const Node kNodeD{kNodeIdD, {kBranchPointD}, {kEdgeIdD}};
  const Graph graph{
      std::unordered_map<EdgeId, Edge>{{kEdgeIdA, kEdgeA}, {kEdgeIdB, kEdgeB}, {kEdgeIdC, kEdgeC}, {kEdgeIdD, kEdgeD}},
      std::unordered_map<NodeId, Node>{{kNodeIdA, kNodeA}, {kNodeIdB, kNodeB}, {kNodeIdC, kNodeC}, {kNodeIdD, kNodeD}}};

  {  // A -> B
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeA, kNodeB);

    ASSERT_EQ(1u, edge_sequences.size());
    const std::vector<Edge>& edge_sequence_1 = edge_sequences[0];
    ASSERT_EQ(1u, edge_sequence_1.size());
    ASSERT_EQ(kEdgeIdA, edge_sequence_1[0].id);
  }
  // TODO: A -> C and A -> D require a complex creation of mocks to properly evaluate results.
}

// Extends the case of the graph to include one entry and one exit to the diverging-merging path:
//
// <pre>
//
//    S:A
// A x-------x B
//    S:B
// C x-------x D
//
// </pre>
GTEST_TEST(FindAllEdgeSequences, DisjointGraph) {
  const EdgeId kEdgeIdA("edge_a");
  const EdgeId kEdgeIdB("edge_b");
  const api::Segment* kSegmentA{reinterpret_cast<const api::Segment*>(0x00000001)};
  const api::Segment* kSegmentB{reinterpret_cast<const api::Segment*>(0x00000002)};
  const NodeId kNodeIdA("0");
  const NodeId kNodeIdB("1");
  const NodeId kNodeIdC("2");
  const NodeId kNodeIdD("3");
  const api::BranchPoint* kBranchPointA{reinterpret_cast<const api::BranchPoint*>(0x00010001)};
  const api::BranchPoint* kBranchPointB{reinterpret_cast<const api::BranchPoint*>(0x00010002)};
  const api::BranchPoint* kBranchPointC{reinterpret_cast<const api::BranchPoint*>(0x00010003)};
  const api::BranchPoint* kBranchPointD{reinterpret_cast<const api::BranchPoint*>(0x00010004)};
  const Edge kEdgeA{kEdgeIdA, kSegmentA, kNodeIdA, kNodeIdB};
  const Edge kEdgeB{kEdgeIdB, kSegmentB, kNodeIdC, kNodeIdD};
  const Node kNodeA{kNodeIdA, {kBranchPointA}, {kEdgeIdA}};
  const Node kNodeB{kNodeIdB, {kBranchPointB}, {kEdgeIdA}};
  const Node kNodeC{kNodeIdC, {kBranchPointC}, {kEdgeIdB}};
  const Node kNodeD{kNodeIdD, {kBranchPointD}, {kEdgeIdB}};
  const Graph graph{
      std::unordered_map<EdgeId, Edge>{{kEdgeIdA, kEdgeA}, {kEdgeIdB, kEdgeB}},
      std::unordered_map<NodeId, Node>{{kNodeIdA, kNodeA}, {kNodeIdB, kNodeB}, {kNodeIdC, kNodeC}, {kNodeIdD, kNodeD}}};

  {  // A -> B
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeA, kNodeB);

    ASSERT_EQ(1u, edge_sequences.size());
    const std::vector<Edge>& edge_sequence_1 = edge_sequences[0];
    ASSERT_EQ(1u, edge_sequence_1.size());
    ASSERT_EQ(kEdgeIdA, edge_sequence_1[0].id);
  }
  {  // C -> D
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeC, kNodeD);

    ASSERT_EQ(1u, edge_sequences.size());
    const std::vector<Edge>& edge_sequence_1 = edge_sequences[0];
    ASSERT_EQ(1u, edge_sequence_1.size());
    ASSERT_EQ(kEdgeIdB, edge_sequence_1[0].id);
  }
  {  // A -> C
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeA, kNodeC);

    ASSERT_TRUE(edge_sequences.empty());
  }
  {  // B -> D
    const std::vector<std::vector<Edge>> edge_sequences = FindAllEdgeSequences(graph, kNodeB, kNodeD);

    ASSERT_TRUE(edge_sequences.empty());
  }
}

// Evaluates that the same position but changing the end yields two different nodes.
GTEST_TEST(FindNode, CorrectlyReturnsTheNodeWhenFound) {
  static constexpr int kOne{1};
  JunctionMock junction;
  SegmentMock segment;
  const api::SegmentId kSegmentId("segment_id");
  const api::Segment* segment_ptr = &segment;
  LaneMock lane;
  LaneMock strange_lane;
  BranchPointMock start_branch_point;
  const api::BranchPoint* start_branch_point_ptr = &start_branch_point;
  BranchPointMock end_branch_point;
  const api::BranchPoint* end_branch_point_ptr = &end_branch_point;
  BranchPointMock strange_branch_point;
  const api::BranchPoint* strange_branch_point_ptr = &strange_branch_point;
  RoadGeometryMock rg;
  const api::RoadPosition position{&lane, api::LanePosition{1., 2., 3.}};
  const api::RoadPosition strange_position{&strange_lane, api::LanePosition{1., 2., 3.}};
  EXPECT_CALL(rg, do_num_junctions()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(rg, do_junction(_)).WillRepeatedly(Return(static_cast<const api::Junction*>(&junction)));
  EXPECT_CALL(junction, do_num_segments()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(junction, do_segment(_)).WillRepeatedly(Return(segment_ptr));
  EXPECT_CALL(segment, do_id()).WillRepeatedly(Return(kSegmentId));
  EXPECT_CALL(segment, do_num_lanes()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(segment, do_lane(_)).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane)));
  EXPECT_CALL(lane, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart))).WillRepeatedly(Return(start_branch_point_ptr));
  EXPECT_CALL(lane, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish))).WillRepeatedly(Return(end_branch_point_ptr));
  EXPECT_CALL(strange_lane, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart)))
      .WillRepeatedly(Return(strange_branch_point_ptr));
  const Graph graph = BuildGraph(&rg);

  const std::optional<Node> expected_strange_node = FindNode(graph, strange_position, api::LaneEnd::Which::kStart);
  const std::optional<Node> expected_start_node = FindNode(graph, position, api::LaneEnd::Which::kStart);
  const std::optional<Node> expected_end_node = FindNode(graph, position, api::LaneEnd::Which::kFinish);

  // This position does not map to an api::BranchPoint in the graph, so it is strange in the context of this graph.
  ASSERT_FALSE(expected_strange_node.has_value());
  // The following two positions map to api::BranchPoints in the graph.
  ASSERT_TRUE(expected_start_node.has_value());
  ASSERT_TRUE(expected_end_node.has_value());
  ASSERT_NE(expected_start_node->id, expected_end_node->id);
  ASSERT_TRUE(
      expected_start_node->branch_points.find(start_branch_point_ptr) != expected_start_node->branch_points.end() ||
      expected_start_node->branch_points.find(end_branch_point_ptr) != expected_start_node->branch_points.end());
  ASSERT_TRUE(expected_end_node->branch_points.find(start_branch_point_ptr) != expected_end_node->branch_points.end() ||
              expected_end_node->branch_points.find(end_branch_point_ptr) != expected_end_node->branch_points.end());
  ASSERT_EQ(EdgeId(kSegmentId.string()), *(expected_start_node->edges.begin()));
  ASSERT_EQ(EdgeId(kSegmentId.string()), *(expected_end_node->edges.begin()));
}

// Extends the case of the graph to include one entry and one exit to the diverging-merging path:
//
// <pre>
//
//    S:A    S:B
// x-------x-------x
// A       B       C
//    S:C
// x-------x
// D       E
//
// </pre>
GTEST_TEST(DetermineEdgeEnd, EvaluateJointAndDisjointEdges) {
  const EdgeId kEdgeIdA("edge_a");
  const EdgeId kEdgeIdB("edge_b");
  const EdgeId kEdgeIdC("edge_c");
  const api::Segment* kSegmentA{reinterpret_cast<const api::Segment*>(0x00000001)};
  const api::Segment* kSegmentB{reinterpret_cast<const api::Segment*>(0x00000002)};
  const api::Segment* kSegmentC{reinterpret_cast<const api::Segment*>(0x00000003)};
  const NodeId kNodeIdA("0");
  const NodeId kNodeIdB("1");
  const NodeId kNodeIdC("2");
  const NodeId kNodeIdD("3");
  const NodeId kNodeIdE("4");
  const api::BranchPoint* kBranchPointA{reinterpret_cast<const api::BranchPoint*>(0x00010001)};
  const api::BranchPoint* kBranchPointB{reinterpret_cast<const api::BranchPoint*>(0x00010002)};
  const api::BranchPoint* kBranchPointC{reinterpret_cast<const api::BranchPoint*>(0x00010003)};
  const api::BranchPoint* kBranchPointD{reinterpret_cast<const api::BranchPoint*>(0x00010004)};
  const api::BranchPoint* kBranchPointE{reinterpret_cast<const api::BranchPoint*>(0x00010005)};
  const Edge kEdgeA{kEdgeIdA, kSegmentA, kNodeIdA, kNodeIdB};
  const Edge kEdgeB{kEdgeIdB, kSegmentB, kNodeIdB, kNodeIdC};
  const Edge kEdgeC{kEdgeIdC, kSegmentC, kNodeIdD, kNodeIdE};
  const Node kNodeA{kNodeIdA, {kBranchPointA}, {kEdgeIdA}};
  const Node kNodeB{kNodeIdB, {kBranchPointB}, {kEdgeIdA, kEdgeIdB}};
  const Node kNodeC{kNodeIdC, {kBranchPointC}, {kEdgeIdB}};
  const Node kNodeD{kNodeIdD, {kBranchPointD}, {kEdgeIdC}};
  const Node kNodeE{kNodeIdE, {kBranchPointE}, {kEdgeIdC}};
  const Graph graph{
      std::unordered_map<EdgeId, Edge>{{kEdgeIdA, kEdgeA}, {kEdgeIdB, kEdgeB}, {kEdgeIdB, kEdgeC}},
      std::unordered_map<NodeId, Node>{
          {kNodeIdA, kNodeA}, {kNodeIdB, kNodeB}, {kNodeIdC, kNodeC}, {kNodeIdD, kNodeD}, {kNodeIdE, kNodeE}}};

  {  // A -> B
    const std::optional<api::LaneEnd::Which> end = DetermineEdgeEnd(graph, kEdgeA, kEdgeB);

    ASSERT_TRUE(end.has_value());
    ASSERT_EQ(api::LaneEnd::Which::kFinish, end.value());
  }
  {  // B -> A
    const std::optional<api::LaneEnd::Which> end = DetermineEdgeEnd(graph, kEdgeB, kEdgeA);

    ASSERT_TRUE(end.has_value());
    ASSERT_EQ(api::LaneEnd::Which::kStart, end.value());
  }
  {  // C -> C
    const std::optional<api::LaneEnd::Which> end = DetermineEdgeEnd(graph, kEdgeC, kEdgeC);

    ASSERT_FALSE(end.has_value());
  }
  {  // A -> C
    const std::optional<api::LaneEnd::Which> end = DetermineEdgeEnd(graph, kEdgeA, kEdgeC);

    ASSERT_FALSE(end.has_value());
  }
}

}  // namespace
}  // namespace test
}  // namespace graph
}  // namespace routing
}  // namespace maliput
