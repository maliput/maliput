// BSD 3-Clause License
//
// Copyright (c) 2023-2026, Woven by Toyota.
// All rights reserved.
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
#include "maliput/api/compare.h"

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "maliput/api/lane.h"

namespace maliput {
namespace api {
namespace {

template <typename T>
common::ComparisonResult<T> IsEqual(const char* object_type, const char* a_expression, const char* b_expression,
                                    const T* a, const T* b) {
  if (a != b) {
    return {"Pointers are referenced to different " + std::string(object_type) + " objects. " +
            std::string(a_expression) + " vs. " + std::string(b_expression) + "\n"};
  }
  return {std::nullopt};
}

}  // namespace

common::ComparisonResult<InertialPosition> IsInertialPositionClose(const InertialPosition& pos1,
                                                                   const InertialPosition& pos2, double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(pos1.x() - pos2.x());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "InertialPositions are different at x coordinate. " +
                    "pos1.x(): " + std::to_string(pos1.x()) + " vs. " + "pos2.x(): " + std::to_string(pos2.x()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(pos1.y() - pos2.y());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "InertialPositions are different at y coordinate. " +
                    "pos1.y(): " + std::to_string(pos1.y()) + " vs. " + "pos2.y(): " + std::to_string(pos2.y()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(pos1.z() - pos2.z());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "InertialPositions are different at z coordinate. " +
                    "pos1.z(): " + std::to_string(pos1.z()) + " vs. " + "pos2.z(): " + std::to_string(pos2.z()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return {error_message};
  }
  return {std::nullopt};
}

common::ComparisonResult<LanePosition> IsLanePositionClose(const LanePosition& pos1, const LanePosition& pos2,
                                                           double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(pos1.s() - pos2.s());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "LanePositions are different at s coordinate. " +
                    "pos1.s(): " + std::to_string(pos1.s()) + " vs. " + "pos2.s(): " + std::to_string(pos2.s()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(pos1.r() - pos2.r());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "LanePositions are different at r coordinate. " +
                    "pos1.r(): " + std::to_string(pos1.r()) + " vs. " + "pos2.r(): " + std::to_string(pos2.r()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(pos1.h() - pos2.h());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "LanePositions are different at h coordinate. " +
                    "pos1.h(): " + std::to_string(pos1.h()) + " vs. " + "pos2.h(): " + std::to_string(pos2.h()) +
                    ", diff = " + std::to_string(delta) + ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return {error_message};
  }
  return {std::nullopt};
}

common::ComparisonResult<Rotation> IsRotationClose(const Rotation& rot1, const Rotation& rot2, double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(rot1.roll() - rot2.roll());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Rotations are different at roll angle. " +
                    "rot1.roll(): " + std::to_string(rot1.roll()) + " vs. " +
                    "rot2.roll(): " + std::to_string(rot2.roll()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(rot1.pitch() - rot2.pitch());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Rotations are different at pitch angle. " +
                    "rot1.pitch(): " + std::to_string(rot1.pitch()) + " vs. " +
                    "rot2.pitch(): " + std::to_string(rot2.pitch()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(rot1.yaw() - rot2.yaw());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "Rotations are different at yaw angle. " +
                    "rot1.yaw(): " + std::to_string(rot1.yaw()) + " vs. " +
                    "rot2.yaw(): " + std::to_string(rot2.yaw()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return {error_message};
  }
  return {std::nullopt};
}

common::ComparisonResult<RBounds> IsRBoundsClose(const RBounds& rbounds1, const RBounds& rbounds2, double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(rbounds1.min() - rbounds2.min());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "RBounds are different at r_min. " +
                    "rbounds1.r_min: " + std::to_string(rbounds1.min()) +
                    " vs. rbounds2.r_min: " + std::to_string(rbounds2.min()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(rbounds1.max() - rbounds2.max());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "RBounds are different at r_max. " +
                    "rbounds1.r_max: " + std::to_string(rbounds1.max()) +
                    " vs. rbounds2.r_max: " + std::to_string(rbounds2.max()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return {error_message};
  }
  return {std::nullopt};
}

common::ComparisonResult<HBounds> IsHBoundsClose(const HBounds& hbounds1, const HBounds& hbounds2, double tolerance) {
  bool fails = false;
  std::string error_message;
  double delta = std::abs(hbounds1.min() - hbounds2.min());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "HBounds are different at min. rbounds1.min(): " + std::to_string(hbounds1.min()) +
                    " vs. rbounds2.min(): " + std::to_string(hbounds2.min()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  delta = std::abs(hbounds1.max() - hbounds2.max());
  if (delta > tolerance) {
    fails = true;
    error_message = error_message + "HBounds are different at max. rbounds1.max(): " + std::to_string(hbounds1.max()) +
                    " vs. rbounds2.max(): " + std::to_string(hbounds2.max()) + ", diff = " + std::to_string(delta) +
                    ", tolerance = " + std::to_string(tolerance) + "\n";
  }
  if (fails) {
    return {error_message};
  }
  return {std::nullopt};
}

common::ComparisonResult<LanePositionResult> IsLanePositionResultClose(const LanePositionResult& lpr_a,
                                                                       const LanePositionResult& lpr_b,
                                                                       double tolerance) {
  common::ComparisonResult<LanePosition> lane_position_result =
      IsLanePositionClose(lpr_a.lane_position, lpr_b.lane_position, tolerance);
  if (lane_position_result.message.has_value()) return {lane_position_result.message};

  common::ComparisonResult<InertialPosition> nearest_position_result =
      IsInertialPositionClose(lpr_a.nearest_position, lpr_b.nearest_position, tolerance);
  if (nearest_position_result.message.has_value()) return {nearest_position_result.message};

  const double delta = std::abs(lpr_a.distance - lpr_b.distance);
  if (delta > tolerance) {
    return {"LanePositionResult are different at distance. lpr_a.distance: " + std::to_string(lpr_a.distance) +
            " vs. lpr_b.distance: " + std::to_string(lpr_b.distance) + ", diff = " + std::to_string(delta) +
            ", tolerance = " + std::to_string(tolerance) + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<RoadPositionResult> IsRoadPositionResultClose(const maliput::api::RoadPositionResult& rpr_a,
                                                                       const maliput::api::RoadPositionResult& rpr_b,
                                                                       double tolerance) {
  if (rpr_a.road_position.lane == nullptr) {
    return {"Lane of rpr_a is nullptr"};
  }
  if (rpr_b.road_position.lane == nullptr) {
    return {"Lane of rpr_b is nullptr"};
  }
  if (rpr_a.road_position.lane != rpr_b.road_position.lane) {
    return {"RoadPositionResult are different at road_position.lane: rpr_a.road_position.lane: " +
            rpr_a.road_position.lane->id().string() +
            " vs. rpr_b.road_position.lane: " + rpr_b.road_position.lane->id().string() + "\n"};
  }
  return {IsLanePositionResultClose({rpr_a.road_position.pos, rpr_a.nearest_position, rpr_a.distance},
                                    {rpr_b.road_position.pos, rpr_b.nearest_position, rpr_b.distance}, tolerance)
              .message};
}

common::ComparisonResult<LaneEnd> IsLaneEndEqual(const LaneEnd& lane_end1, const LaneEnd& lane_end2) {
  auto which_to_string = [](const LaneEnd::Which end) {
    return end == LaneEnd::Which::kStart ? std::string("kStart") : std::string("kFinish");
  };
  if (lane_end1.lane != lane_end2.lane) {
    return {"lane_end1.lane is different from lane_end2.lane. lane_end1.lane: " + lane_end1.lane->id().string() +
            " vs. lane_end2.lane: " + lane_end2.lane->id().string() + "\n"};
  }
  if (lane_end1.end != lane_end2.end) {
    return {"lane_end1.end is different from lane_end2.end. lane_end1.end: " + which_to_string(lane_end1.end) +
            " vs. lane_end2.end: " + which_to_string(lane_end2.end) + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<bool> IsEqual(const char* a_expression, const char* b_expression, bool a, bool b) {
  if (a != b) {
    return {"Values are different. " + std::string(a_expression) + ": " + std::to_string(a) + " vs. " +
            std::string(b_expression) + ": " + std::to_string(b) + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<double> IsEqual(const char* a_expression, const char* b_expression, double a, double b) {
  const double delta = std::abs(a - b);
  if (delta > std::numeric_limits<double>::epsilon()) {
    return {"Values are different. " + std::string(a_expression) + ": " + std::to_string(a) + " vs. " +
            std::string(b_expression) + ": " + std::to_string(b) + ", diff = " + std::to_string(delta) + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<std::size_t> IsEqual(const char* a_expression, const char* b_expression, std::size_t a,
                                              std::size_t b) {
  if (a != b) {
    return {"Values are different. " + std::string(a_expression) + ": " + std::to_string(a) + " vs. " +
            std::string(b_expression) + ": " + std::to_string(b) + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<SRange> IsEqual(const SRange& s_range_1, const SRange& s_range_2) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, IsEqual("s_range_1.s0()", "s_range_2.s0()", s_range_1.s0(), s_range_2.s0()));
  MALIPUT_ADD_RESULT(c, IsEqual("s_range_1.s1()", "s_range_2.s1()", s_range_1.s1(), s_range_2.s1()));
  return {c.result()};
}

common::ComparisonResult<LaneSRange> IsEqual(const LaneSRange& lane_s_range_1, const LaneSRange& lane_s_range_2) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, IsEqual("lane_s_range_1.lane_id()", "lane_s_range_2.lane_id()", lane_s_range_1.lane_id(),
                                lane_s_range_2.lane_id()));
  MALIPUT_ADD_RESULT(c, IsEqual(lane_s_range_1.s_range(), lane_s_range_2.s_range()));
  return {c.result()};
}

common::ComparisonResult<std::vector<LaneSRange>> IsEqual(const std::vector<LaneSRange>& lane_s_ranges_1,
                                                          const std::vector<LaneSRange>& lane_s_ranges_2) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(
      c, IsEqual("lane_s_ranges_1.size()", "lane_s_ranges_2.size()", lane_s_ranges_1.size(), lane_s_ranges_2.size()));
  const int smallest = std::min(lane_s_ranges_1.size(), lane_s_ranges_2.size());
  for (int i = 0; i < smallest; ++i) {
    MALIPUT_ADD_RESULT(c, IsEqual(lane_s_ranges_1[i], lane_s_ranges_2[i]));
  }
  return {c.result()};
}

common::ComparisonResult<LaneSRoute> IsEqual(const LaneSRoute& lane_s_route_1, const LaneSRoute& lane_s_route_2) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, IsEqual(lane_s_route_1.ranges(), lane_s_route_2.ranges()));
  return {c.result()};
}

common::ComparisonResult<maliput::api::Junction> IsEqual(const char* a_expression, const char* b_expression,
                                                         const maliput::api::Junction* a,
                                                         const maliput::api::Junction* b) {
  return IsEqual<maliput::api::Junction>("Junction", a_expression, b_expression, a, b);
}

common::ComparisonResult<Segment> IsEqual(const char* a_expression, const char* b_expression, const Segment* a,
                                          const Segment* b) {
  return IsEqual<Segment>("Segment", a_expression, b_expression, a, b);
}

common::ComparisonResult<Lane> IsEqual(const char* a_expression, const char* b_expression, const Lane* a,
                                       const Lane* b) {
  return IsEqual<Lane>("Lane", a_expression, b_expression, a, b);
}

common::ComparisonResult<BranchPoint> IsEqual(const char* a_expression, const char* b_expression, const BranchPoint* a,
                                              const BranchPoint* b) {
  return IsEqual<BranchPoint>("BranchPoint", a_expression, b_expression, a, b);
}

std::optional<std::string> CheckIdIndexing(const RoadGeometry* road_geometry) {
  common::ComparisonResultCollector c;
  for (int ji = 0; ji < road_geometry->num_junctions(); ++ji) {
    const api::Junction* junction = road_geometry->junction(ji);
    MALIPUT_ADD_RESULT(
        c, IsEqual("road_geometry->ById().GetJunction(junction->id())", "static_cast<const Junction*>(junction)",
                   road_geometry->ById().GetJunction(junction->id()), junction));
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      MALIPUT_ADD_RESULT(c, IsEqual("road_geometry->ById().GetSegment(segment->id())", "segment",
                                    road_geometry->ById().GetSegment(segment->id()), segment));
      for (int li = 0; li < segment->num_lanes(); ++li) {
        const api::Lane* lane = segment->lane(li);
        MALIPUT_ADD_RESULT(c, IsEqual("road_geometry->ById().GetLane(lane->id())", "lane",
                                      road_geometry->ById().GetLane(lane->id()), lane));
      }
    }
    for (int bi = 0; bi < road_geometry->num_branch_points(); ++bi) {
      const api::BranchPoint* branch_point = road_geometry->branch_point(bi);
      MALIPUT_ADD_RESULT(c, IsEqual("road_geometry->ById().GetBranchPoint(branch_point->id())", "branch_point",
                                    road_geometry->ById().GetBranchPoint(branch_point->id()), branch_point));
    }
  }
  return c.result();
}

common::ComparisonResult<InertialPosition> IsEqual(const InertialPosition& inertial_position_1,
                                                   const InertialPosition& inertial_position_2) {
  if (inertial_position_1 != inertial_position_2) {
    return {std::string("InertialPositions are different. ") +
            "inertial_position_1: " + inertial_position_1.xyz().to_str() +
            " vs. inertial_position_2: " + inertial_position_2.xyz().to_str() + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<Rotation> IsEqual(const Rotation& rotation_1, const Rotation& rotation_2) {
  if (rotation_1.matrix() != rotation_2.matrix()) {
    return {std::string("Rotations are different. ") + "rotation_1: " + rotation_1.matrix().to_str() +
            " vs. rotation_2: " + rotation_2.matrix().to_str() + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<LaneEnd::Which> IsEqual(const LaneEnd::Which& which_1, const LaneEnd::Which& which_2) {
  if (which_1 != which_2) {
    return {std::string("LaneEnd::Which are different. ") + "which_1: " + std::to_string(which_1) +
            " vs. which_2: " + std::to_string(which_2) + "\n"};
  }
  return {std::nullopt};
}

common::ComparisonResult<LaneEnd> IsEqual(const LaneEnd& lane_end_1, const LaneEnd& lane_end_2) {
  common::ComparisonResultCollector c;
  MALIPUT_ADD_RESULT(c, IsEqual("lane_end_1.lane", "lane_end_2.lane", lane_end_1.lane, lane_end_2.lane));
  MALIPUT_ADD_RESULT(c, IsEqual(lane_end_1.end, lane_end_2.end));
  return {c.result()};
}

}  // namespace api
}  // namespace maliput
