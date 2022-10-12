// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput/test_utilities/maliput_types_compare.h"

#include <cmath>
#include <string>

namespace maliput {
namespace api {
namespace test {

::testing::AssertionResult IsInertialPositionClose(const InertialPosition& pos1, const InertialPosition& pos2,
                                                   double tolerance) {
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
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "pos1 =\n"
                                       << pos1 << "\nis approximately equal to pos2 =\n"
                                       << pos2 << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsLanePositionClose(const LanePosition& pos1, const LanePosition& pos2, double tolerance) {
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
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "pos1 =\n"
                                       << pos1 << "\nis approximately equal to pos2 =\n"
                                       << pos2 << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsRotationClose(const Rotation& rot1, const Rotation& rot2, double tolerance) {
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
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "rot1 =\n"
                                       << rot1 << "\nis approximately equal to rot2 =\n"
                                       << rot2 << " within " << tolerance << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsRBoundsClose(const RBounds& rbounds1, const RBounds& rbounds2, double tolerance) {
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
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "rbounds1 =\n"
                                       << "(" << rbounds1.min() << ", " << rbounds1.max() << ")"
                                       << "\nis approximately equal to "
                                       << " rbounds2 =\n"
                                       << "(" << rbounds2.min() << ", " << rbounds2.max() << ")"
                                       << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsHBoundsClose(const HBounds& hbounds1, const HBounds& hbounds2, double tolerance) {
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
    return ::testing::AssertionFailure() << error_message;
  }
  return ::testing::AssertionSuccess() << "hbounds1 =\n"
                                       << "(" << hbounds1.min() << ", " << hbounds1.max() << ")"
                                       << "\nis approximately equal to "
                                       << " hbounds2 =\n"
                                       << "(" << hbounds2.min() << ", " << hbounds2.max() << ")"
                                       << ", tolerance = " << tolerance;
}

::testing::AssertionResult IsLanePositionResultClose(const LanePositionResult& lpr_a, const LanePositionResult& lpr_b,
                                                     double tolerance) {
  ::testing::AssertionResult lane_position_result =
      IsLanePositionClose(lpr_a.lane_position, lpr_b.lane_position, tolerance);
  if (!lane_position_result) return lane_position_result;

  ::testing::AssertionResult nearest_position_result =
      IsInertialPositionClose(lpr_a.nearest_position, lpr_b.nearest_position, tolerance);
  if (!nearest_position_result) return nearest_position_result;

  const double delta = std::abs(lpr_a.distance - lpr_b.distance);
  if (delta > tolerance) {
    return ::testing::AssertionFailure() << "LanePositionResult are different at distance. lpr_a.distance: "
                                         << lpr_a.distance << " vs. lpr_b.distance: " << lpr_b.distance
                                         << ", diff = " << delta << ", tolerance = " << tolerance << "\n";
  }
  return ::testing::AssertionSuccess();
}

testing::AssertionResult IsRoadPositionResultClose(const maliput::api::RoadPositionResult& rpr_a,
                                                   const maliput::api::RoadPositionResult& rpr_b, double tolerance) {
  if (rpr_a.road_position.lane != rpr_b.road_position.lane) {
    return testing::AssertionFailure()
           << "RoadPositionResult are different at road_position.lane: rpr_a.road_position.lane: "
           << rpr_a.road_position.lane << " vs. rpr_b.road_position.lane: " << rpr_b.road_position.lane;
  }
  return IsLanePositionResultClose({rpr_a.road_position.pos, rpr_a.nearest_position, rpr_a.distance},
                                   {rpr_b.road_position.pos, rpr_b.nearest_position, rpr_b.distance}, tolerance);
}

::testing::AssertionResult IsLaneEndEqual(const LaneEnd& lane_end1, const LaneEnd& lane_end2) {
  auto which_to_string = [](const LaneEnd::Which end) {
    return end == LaneEnd::Which::kStart ? std::string("kStart") : std::string("kFinish");
  };
  if (lane_end1.lane != lane_end2.lane) {
    return ::testing::AssertionFailure() << "lane_end1.lane is different from lane_end2.lane. lane_end1.lane: "
                                         << lane_end1.lane << " vs. lane_end2.lane: " << lane_end2.lane << "\n";
  }
  if (lane_end1.end != lane_end2.end) {
    return ::testing::AssertionFailure() << "lane_end1.end is different from lane_end2.end. lane_end1.end: "
                                         << which_to_string(lane_end1.end)
                                         << " vs. lane_end2.end: " << which_to_string(lane_end2.end) << "\n";
  }
  return ::testing::AssertionSuccess();
}

}  // namespace test
}  // namespace api
}  // namespace maliput
