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

#include <set>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "maliput/api/branch_point.h"
#include "maliput/api/segment.h"
#include "maliput/common/assertion_error.h"
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

GTEST_TEST(BuildGraph, PassingNullRoadGeometryThrows) {
  ASSERT_THROW({ BuildGraph(nullptr); }, common::assertion_error);
}

// The graph represents the following structure:
// One Junction - one segment - one lane (-)
// Two BranchPoints (x)
//
// x------------x
GTEST_TEST(BuildGraph, EvaluateSingleLaneRoadGraph) {
  static constexpr int kOne{1};
  JunctionMock junction;
  SegmentMock segment;
  const api::Segment* segment_ptr = &segment;
  LaneMock lane;
  BranchPointMock start_branch_point;
  const api::BranchPoint* start_branch_point_ptr = &start_branch_point;
  BranchPointMock end_branch_point;
  const api::BranchPoint* end_branch_point_ptr = &end_branch_point;
  RoadGeometryMock rg;
  EXPECT_CALL(rg, do_num_junctions()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(rg, do_junction(_)).WillRepeatedly(Return(static_cast<const api::Junction*>(&junction)));
  EXPECT_CALL(junction, do_num_segments()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(junction, do_segment(_)).WillRepeatedly(Return(segment_ptr));
  EXPECT_CALL(segment, do_num_lanes()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(segment, do_lane(_)).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane)));
  EXPECT_CALL(lane, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart))).WillRepeatedly(Return(start_branch_point_ptr));
  EXPECT_CALL(lane, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish))).WillRepeatedly(Return(end_branch_point_ptr));

  const Graph graph = BuildGraph(&rg);

  ASSERT_EQ(1u, graph.edges.size());
  ASSERT_EQ(2u, graph.nodes.size());
  ASSERT_NE(graph.edges.end(), graph.edges.find(segment_ptr));
  const Edge& edge = graph.edges.at(segment_ptr);
  ASSERT_EQ(segment_ptr, edge.segment);
  ASSERT_NE(edge.node_a, edge.node_b);
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge.node_a));
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge.node_b));
  const Node& node_a = graph.nodes.find(edge.node_a)->second;
  const Node& node_b = graph.nodes.find(edge.node_b)->second;
  ASSERT_EQ(1u, node_a.branch_points.size());
  ASSERT_NE(node_a.branch_points.end(), node_a.branch_points.find(start_branch_point_ptr));
  ASSERT_EQ(1u, node_a.edges.size());
  ASSERT_NE(node_a.edges.end(), node_a.edges.find(segment_ptr));
  ASSERT_EQ(1u, node_b.branch_points.size());
  ASSERT_NE(node_b.branch_points.end(), node_b.branch_points.find(end_branch_point_ptr));
  ASSERT_EQ(1u, node_b.edges.size());
  ASSERT_NE(node_b.edges.end(), node_b.edges.find(segment_ptr));
}

// The graph represents the following structure:
// One Junction - one segment - two lanes (-)
// Four BranchPoints (x)
//
//       a
// x------------x
// x------------x
//       b
GTEST_TEST(BuildGraph, EvaluateDoubleLaneRoadGraph) {
  static constexpr int kOne{1};
  static constexpr int kTwo{2};
  JunctionMock junction;
  SegmentMock segment;
  const api::Segment* segment_ptr = &segment;
  LaneMock lane_a;
  BranchPointMock start_branch_point_a;
  const api::BranchPoint* start_branch_point_a_ptr = &start_branch_point_a;
  BranchPointMock end_branch_point_a;
  const api::BranchPoint* end_branch_point_a_ptr = &end_branch_point_a;
  LaneMock lane_b;
  BranchPointMock start_branch_point_b;
  const api::BranchPoint* start_branch_point_b_ptr = &start_branch_point_b;
  BranchPointMock end_branch_point_b;
  const api::BranchPoint* end_branch_point_b_ptr = &end_branch_point_b;
  RoadGeometryMock rg;
  EXPECT_CALL(rg, do_num_junctions()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(rg, do_junction(_)).WillRepeatedly(Return(static_cast<const api::Junction*>(&junction)));
  EXPECT_CALL(junction, do_num_segments()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(junction, do_segment(_)).WillRepeatedly(Return(segment_ptr));
  EXPECT_CALL(segment, do_num_lanes()).WillRepeatedly(Return(kTwo));
  EXPECT_CALL(segment, do_lane(Eq(0))).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane_a)));
  EXPECT_CALL(segment, do_lane(Eq(1))).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane_b)));
  EXPECT_CALL(lane_a, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart)))
      .WillRepeatedly(Return(start_branch_point_a_ptr));
  EXPECT_CALL(lane_a, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish)))
      .WillRepeatedly(Return(end_branch_point_a_ptr));
  EXPECT_CALL(lane_b, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart)))
      .WillRepeatedly(Return(start_branch_point_b_ptr));
  EXPECT_CALL(lane_b, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish)))
      .WillRepeatedly(Return(end_branch_point_b_ptr));

  const Graph graph = BuildGraph(&rg);

  ASSERT_EQ(1u, graph.edges.size());
  ASSERT_EQ(2u, graph.nodes.size());
  ASSERT_NE(graph.edges.end(), graph.edges.find(segment_ptr));
  const Edge& edge = graph.edges.at(segment_ptr);
  ASSERT_EQ(segment_ptr, edge.segment);
  ASSERT_NE(edge.node_a, edge.node_b);
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge.node_a));
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge.node_b));
  const Node& node_a = graph.nodes.find(edge.node_a)->second;
  const Node& node_b = graph.nodes.find(edge.node_b)->second;
  ASSERT_EQ(2u, node_a.branch_points.size());
  ASSERT_NE(node_a.branch_points.end(), node_a.branch_points.find(start_branch_point_a_ptr));
  ASSERT_NE(node_a.branch_points.end(), node_a.branch_points.find(start_branch_point_b_ptr));
  ASSERT_EQ(1u, node_a.edges.size());
  ASSERT_NE(node_a.edges.end(), node_a.edges.find(segment_ptr));
  ASSERT_EQ(2u, node_b.branch_points.size());
  ASSERT_NE(node_b.branch_points.end(), node_b.branch_points.find(end_branch_point_a_ptr));
  ASSERT_NE(node_b.branch_points.end(), node_b.branch_points.find(end_branch_point_b_ptr));
  ASSERT_EQ(1u, node_b.edges.size());
  ASSERT_NE(node_b.edges.end(), node_b.edges.find(segment_ptr));
}

// One Junction:
//  - One segment (left to right) - two lanes
//  - One segment (going up) - one lane
//  - One segment (going down) - two lanes
// Seven BranchPoints (x)
//
//                    x e
//                   / b_a
//          a_a     /
// a x------------x/\   < b
// c x------------x\ \  < d
//          a_b     \ \ 
//                   \ \ c_a
//              c_b   \ \ 
//                     x x
//                     g f
GTEST_TEST(BuildGraph, EvaluateYShapedRoadsGraph) {
  static constexpr int kOne{1};
  static constexpr int kTwo{2};
  static constexpr int kThree{3};
  JunctionMock junction;
  SegmentMock segment_a;
  const api::Segment* segment_a_ptr = &segment_a;
  LaneMock lane_a_a;
  LaneMock lane_a_b;
  SegmentMock segment_b;
  const api::Segment* segment_b_ptr = &segment_b;
  LaneMock lane_b_a;
  SegmentMock segment_c;
  const api::Segment* segment_c_ptr = &segment_c;
  LaneMock lane_c_a;
  LaneMock lane_c_b;
  BranchPointMock branch_point_a;
  BranchPointMock branch_point_b;
  BranchPointMock branch_point_c;
  BranchPointMock branch_point_d;
  BranchPointMock branch_point_e;
  BranchPointMock branch_point_f;
  BranchPointMock branch_point_g;
  const api::BranchPoint* start_branch_point_a_a_ptr = &branch_point_a;
  const api::BranchPoint* end_branch_point_a_a_ptr = &branch_point_b;
  const api::BranchPoint* start_branch_point_a_b_ptr = &branch_point_c;
  const api::BranchPoint* end_branch_point_a_b_ptr = &branch_point_d;
  const api::BranchPoint* start_branch_point_b_a_ptr = &branch_point_b;
  const api::BranchPoint* end_branch_point_b_a_ptr = &branch_point_e;
  const api::BranchPoint* start_branch_point_c_a_ptr = &branch_point_b;
  const api::BranchPoint* end_branch_point_c_a_ptr = &branch_point_f;
  const api::BranchPoint* start_branch_point_c_b_ptr = &branch_point_d;
  const api::BranchPoint* end_branch_point_c_b_ptr = &branch_point_g;
  RoadGeometryMock rg;
  EXPECT_CALL(rg, do_num_junctions()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(rg, do_junction(_)).WillRepeatedly(Return(static_cast<const api::Junction*>(&junction)));
  EXPECT_CALL(junction, do_num_segments()).WillRepeatedly(Return(kThree));
  EXPECT_CALL(junction, do_segment(Eq(0))).WillRepeatedly(Return(segment_a_ptr));
  EXPECT_CALL(junction, do_segment(Eq(1))).WillRepeatedly(Return(segment_b_ptr));
  EXPECT_CALL(junction, do_segment(Eq(2))).WillRepeatedly(Return(segment_c_ptr));
  EXPECT_CALL(segment_a, do_num_lanes()).WillRepeatedly(Return(kTwo));
  EXPECT_CALL(segment_a, do_lane(Eq(0))).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane_a_a)));
  EXPECT_CALL(segment_a, do_lane(Eq(1))).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane_a_b)));
  EXPECT_CALL(segment_b, do_num_lanes()).WillRepeatedly(Return(kOne));
  EXPECT_CALL(segment_b, do_lane(Eq(0))).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane_b_a)));
  EXPECT_CALL(segment_c, do_num_lanes()).WillRepeatedly(Return(kTwo));
  EXPECT_CALL(segment_c, do_lane(Eq(0))).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane_c_a)));
  EXPECT_CALL(segment_c, do_lane(Eq(1))).WillRepeatedly(Return(static_cast<const api::Lane*>(&lane_c_b)));
  EXPECT_CALL(lane_a_a, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart)))
      .WillRepeatedly(Return(start_branch_point_a_a_ptr));
  EXPECT_CALL(lane_a_a, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish)))
      .WillRepeatedly(Return(end_branch_point_a_a_ptr));
  EXPECT_CALL(lane_a_b, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart)))
      .WillRepeatedly(Return(start_branch_point_a_b_ptr));
  EXPECT_CALL(lane_a_b, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish)))
      .WillRepeatedly(Return(end_branch_point_a_b_ptr));
  EXPECT_CALL(lane_b_a, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart)))
      .WillRepeatedly(Return(start_branch_point_b_a_ptr));
  EXPECT_CALL(lane_b_a, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish)))
      .WillRepeatedly(Return(end_branch_point_b_a_ptr));
  EXPECT_CALL(lane_c_a, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart)))
      .WillRepeatedly(Return(start_branch_point_c_a_ptr));
  EXPECT_CALL(lane_c_a, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish)))
      .WillRepeatedly(Return(end_branch_point_c_a_ptr));
  EXPECT_CALL(lane_c_b, DoGetBranchPoint(Eq(api::LaneEnd::Which::kStart)))
      .WillRepeatedly(Return(start_branch_point_c_b_ptr));
  EXPECT_CALL(lane_c_b, DoGetBranchPoint(Eq(api::LaneEnd::Which::kFinish)))
      .WillRepeatedly(Return(end_branch_point_c_b_ptr));

  const Graph graph = BuildGraph(&rg);

  ASSERT_EQ(3u, graph.edges.size());
  ASSERT_EQ(4u, graph.nodes.size());
  ASSERT_NE(graph.edges.end(), graph.edges.find(segment_a_ptr));
  ASSERT_NE(graph.edges.end(), graph.edges.find(segment_b_ptr));
  ASSERT_NE(graph.edges.end(), graph.edges.find(segment_c_ptr));
  const Edge& edge_a = graph.edges.at(segment_a_ptr);
  const Edge& edge_b = graph.edges.at(segment_b_ptr);
  const Edge& edge_c = graph.edges.at(segment_c_ptr);
  ASSERT_EQ(segment_a_ptr, edge_a.segment);
  ASSERT_NE(edge_a.node_a, edge_a.node_b);
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge_a.node_a));
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge_a.node_b));
  const Node& node_a_a = graph.nodes.find(edge_a.node_a)->second;
  const Node& node_a_b = graph.nodes.find(edge_a.node_b)->second;
  ASSERT_EQ(segment_b_ptr, edge_b.segment);
  ASSERT_NE(edge_b.node_a, edge_b.node_b);
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge_b.node_a));
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge_b.node_b));
  const Node& node_b_a = graph.nodes.find(edge_b.node_a)->second;
  const Node& node_b_b = graph.nodes.find(edge_b.node_b)->second;
  ASSERT_EQ(segment_c_ptr, edge_c.segment);
  ASSERT_NE(edge_c.node_a, edge_c.node_b);
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge_c.node_a));
  ASSERT_NE(graph.nodes.end(), graph.nodes.find(edge_c.node_b));
  const Node& node_c_a = graph.nodes.find(edge_c.node_a)->second;
  const Node& node_c_b = graph.nodes.find(edge_c.node_b)->second;
  ASSERT_NE(node_a_a.id, node_a_b.id);
  ASSERT_NE(node_a_a.id, node_b_b.id);
  ASSERT_NE(node_a_a.id, node_c_b.id);
  ASSERT_NE(node_b_a.id, node_b_b.id);
  ASSERT_NE(node_c_a.id, node_c_b.id);
  ASSERT_EQ(node_a_b.id, node_b_a.id);
  ASSERT_EQ(node_a_b.id, node_c_a.id);
  ASSERT_EQ(2u, node_a_a.branch_points.size());
  ASSERT_NE(node_a_a.branch_points.end(), node_a_a.branch_points.find(start_branch_point_a_a_ptr));
  ASSERT_NE(node_a_a.branch_points.end(), node_a_a.branch_points.find(start_branch_point_a_b_ptr));
  ASSERT_EQ(1u, node_a_a.edges.size());
  ASSERT_NE(node_a_a.edges.end(), node_a_a.edges.find(segment_a_ptr));
  ASSERT_EQ(2u, node_a_b.branch_points.size());
  ASSERT_NE(node_a_b.branch_points.end(), node_a_a.branch_points.find(end_branch_point_a_a_ptr));
  ASSERT_NE(node_a_b.branch_points.end(), node_a_a.branch_points.find(end_branch_point_a_b_ptr));
  ASSERT_EQ(3u, node_a_b.edges.size());
  ASSERT_NE(node_a_b.edges.end(), node_a_b.edges.find(segment_a_ptr));
  ASSERT_NE(node_a_b.edges.end(), node_a_b.edges.find(segment_b_ptr));
  ASSERT_NE(node_a_b.edges.end(), node_a_b.edges.find(segment_c_ptr));
  ASSERT_EQ(1u, node_b_b.branch_points.size());
  ASSERT_NE(node_b_b.branch_points.end(), node_b_a.branch_points.find(end_branch_point_b_a_ptr));
  ASSERT_EQ(1u, node_b_b.edges.size());
  ASSERT_NE(node_b_b.edges.end(), node_b_b.edges.find(segment_b_ptr));
  ASSERT_EQ(2u, node_c_b.branch_points.size());
  ASSERT_NE(node_c_b.branch_points.end(), node_c_b.branch_points.find(end_branch_point_c_a_ptr));
  ASSERT_NE(node_c_b.branch_points.end(), node_c_b.branch_points.find(end_branch_point_c_b_ptr));
  ASSERT_EQ(1u, node_c_b.edges.size());
  ASSERT_NE(node_c_b.edges.end(), node_c_b.edges.find(segment_c_ptr));
}

}  // namespace
}  // namespace test
}  // namespace graph
}  // namespace routing
}  // namespace maliput
