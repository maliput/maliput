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
#include "maliput/api/road_geometry.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "maliput/api/branch_point.h"
#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/segment.h"
#include "maliput/common/maliput_abort.h"

namespace maliput {
namespace api {

namespace {

InertialPosition LaneEndInertialPosition(const LaneEnd& lane_end) {
  return lane_end.lane->ToInertialPosition(
      LanePosition((lane_end.end == LaneEnd::kStart) ? 0. : lane_end.lane->length(), 0., 0.));
}

// Return the s/r/h orientation of the specified LaneEnd when travelling
// *out from* the lane at that end.
Rotation OrientationOutFromLane(const LaneEnd& lane_end) {
  switch (lane_end.end) {
    case LaneEnd::kStart: {
      return lane_end.lane->GetOrientation({0., 0., 0.}).Reverse();
    }
    case LaneEnd::kFinish: {
      return lane_end.lane->GetOrientation({lane_end.lane->length(), 0., 0.});
    }
  }
  MALIPUT_ABORT_MESSAGE("lane_end is neither LaneEnd::kStart nor LaneEnd::kFinish");
}

}  // namespace

std::vector<std::string> RoadGeometry::CheckInvariants() const {
  std::vector<std::string> failures;

  // Verify correctness of back-pointers/indexing in object hierarchy.
  for (int bpi = 0; bpi < num_branch_points(); ++bpi) {
    const BranchPoint* bp = branch_point(bpi);
    if (bp->road_geometry() != this) {
      std::stringstream ss;
      ss << "BranchPoint " << bp->id().string() << " is owned by " << this->id().string() << " (" << this
         << ") but claims to be owned by " << bp->road_geometry()->id().string() << " (" << bp->road_geometry() << ").";
      failures.push_back(ss.str());
    }
  }
  for (int ji = 0; ji < num_junctions(); ++ji) {
    const Junction* jnx = junction(ji);
    if (jnx->road_geometry() != this) {
      std::stringstream ss;
      ss << "Junction " << jnx->id().string() << " is owned by " << this->id().string() << " (" << this
         << ") but claims to be owned by " << jnx->road_geometry()->id().string() << " (" << jnx->road_geometry()
         << ").";
      failures.push_back(ss.str());
    }
    for (int si = 0; si < jnx->num_segments(); ++si) {
      const Segment* seg = jnx->segment(si);
      if (seg->junction() != jnx) {
        std::stringstream ss;
        ss << "Segment " << seg->id().string() << " is owned by " << jnx->id().string() << " (" << jnx
           << ") but claims to be owned by " << seg->junction()->id().string() << " (" << seg->junction() << ").";
        failures.push_back(ss.str());
      }
      for (int li = 0; li < seg->num_lanes(); ++li) {
        const Lane* lane = seg->lane(li);
        if (lane->segment() != seg) {
          std::stringstream ss;
          ss << "Lane " << lane->id().string() << " is owned by " << seg->id().string() << " (" << seg
             << ") but claims to be owned by " << lane->segment()->id().string() << " (" << lane->segment() << ").";
          failures.push_back(ss.str());
        }
        // Currently, only Lane has an index() accessor, because its the only
        // component for which the index is meaningful (e.g., adjacency of
        // lanes).
        if (lane->index() != li) {
          std::stringstream ss;
          ss << "Lane " << lane->id().string() << " has index " << li << " but claims to have index " << lane->index()
             << ".";
          failures.push_back(ss.str());
        }
      }
    }
  }

  // Verify C1 continuity at branch-points (within declared tolerances).
  for (int bpi = 0; bpi < num_branch_points(); ++bpi) {
    const BranchPoint* bp = branch_point(bpi);
    // For each BranchPoint:
    //  - all branches should map to same `Inertial`-frame (x,y,z);
    //  - orientation *into* BranchPoint should be the same for all A-side
    //     branches;
    //  - orientation *into* BranchPoint should be the same for all B-side
    //     branches;
    //  - orientation *into* BranchPoint for A-side should be same as
    //     orientation *out of* BranchPoint for B-side.
    if ((bp->GetASide()->size() == 0) && (bp->GetBSide()->size() == 0)) {
      std::stringstream ss;
      ss << "BranchPoint " << bp->id().string() << " is empty.";
      failures.push_back(ss.str());
      continue;
    }

    const LaneEnd ref_end = (bp->GetASide()->size() > 0) ? bp->GetASide()->get(0) : bp->GetBSide()->get(0);
    // ...test `Inertial`-frame position similarity.
    const InertialPosition ref_geo = LaneEndInertialPosition(ref_end);
    const auto test_inertial_position = [&](const LaneEndSet& ends) {
      for (int bi = 0; bi < ends.size(); ++bi) {
        const LaneEnd le = ends.get(bi);
        const double d = ref_geo.Distance(LaneEndInertialPosition(le));
        if (d > linear_tolerance()) {
          std::stringstream ss;
          ss << "Lane " << le.lane->id().string() << ((le.end == LaneEnd::kStart) ? "[start]" : "[end]")
             << " position is off by " << d << " from Lane " << ref_end.lane->id().string()
             << ((ref_end.end == LaneEnd::kStart) ? "[start]" : "[end]");
          failures.push_back(ss.str());
        }
      }
    };
    test_inertial_position(*(bp->GetASide()));
    test_inertial_position(*(bp->GetBSide()));
    // ...test orientation similarity.
    const LaneEnd ref_end_rot = (bp->GetASide()->size() > 0) ? bp->GetASide()->get(0) : bp->GetBSide()->get(0);
    const Rotation ref_rot = (bp->GetASide()->size() > 0) ? OrientationOutFromLane(ref_end_rot)
                                                          : OrientationOutFromLane(ref_end_rot).Reverse();
    const auto test_orientation = [&](const LaneEndSet& ends, const Rotation& reference) {
      for (int bi = 0; bi < ends.size(); ++bi) {
        const LaneEnd le = ends.get(bi);
        const double d = reference.Distance(OrientationOutFromLane(le));
        if (d > angular_tolerance()) {
          std::stringstream ss;
          ss << "Lane " << le.lane->id().string() << ((le.end == LaneEnd::kStart) ? "[start]" : "[end]")
             << " orientation is off by " << d << " from Lane " << ref_end_rot.lane->id().string()
             << ((ref_end_rot.end == LaneEnd::kStart) ? "[start]" : "[end]");
          failures.push_back(ss.str());
        }
      }
    };
    test_orientation(*(bp->GetASide()), ref_rot);
    test_orientation(*(bp->GetBSide()), ref_rot.Reverse());
  }

  // Check that Lane left/right relationships within a Segment are
  // geometrically sound.
  // TODO(maddog@tri.global)  Implement this.

  return failures;
}

std::vector<InertialPosition> RoadGeometry::DoSampleAheadWaypoints(const LaneSRoute& route,
                                                                   double path_length_sampling_rate) const {
  MALIPUT_THROW_UNLESS(path_length_sampling_rate > 0.);
  path_length_sampling_rate = std::max(linear_tolerance(), std::min(path_length_sampling_rate, route.length()));
  std::vector<InertialPosition> waypoints;
  const RoadGeometry::IdIndex& id = ById();
  const std::vector<LaneSRange>& ranges = route.ranges();

  /// Sample first point
  const Lane* first_lane = id.GetLane(ranges.front().lane_id());
  MALIPUT_THROW_UNLESS(first_lane != nullptr);
  waypoints.emplace_back(first_lane->ToInertialPosition(LanePosition(ranges.front().s_range().s0(), 0.0, 0.0)));

  double previous_s_difference = 0.0;
  for (const auto& range : ranges) {
    const Lane* lane = id.GetLane(range.lane_id());
    MALIPUT_THROW_UNLESS(lane != nullptr);
    const SRange lane_s_range = range.s_range();

    double step_accumulator = previous_s_difference + lane_s_range.s0() + path_length_sampling_rate;

    while (step_accumulator <= lane_s_range.s1()) {
      waypoints.emplace_back(lane->ToInertialPosition(LanePosition(step_accumulator, 0.0, 0.0)));
      step_accumulator += path_length_sampling_rate;
    }
    previous_s_difference = step_accumulator - lane_s_range.s1() - path_length_sampling_rate;
  }
  if (std::abs(previous_s_difference) > linear_tolerance()) {
    const Lane* last_lane = id.GetLane(ranges.back().lane_id());
    MALIPUT_THROW_UNLESS(last_lane != nullptr);
    waypoints.emplace_back(last_lane->ToInertialPosition(LanePosition(ranges.back().s_range().s1(), 0.0, 0.0)));
  }
  return waypoints;
}

}  // namespace api
}  // namespace maliput
