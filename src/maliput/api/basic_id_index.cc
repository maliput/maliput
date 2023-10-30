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
#include "maliput/api/basic_id_index.h"

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {

void BasicIdIndex::AddLane(const Lane* lane) { MALIPUT_THROW_UNLESS(lane_map_.emplace(lane->id(), lane).second); }

void BasicIdIndex::AddSegment(const Segment* segment) {
  MALIPUT_THROW_UNLESS(segment_map_.emplace(segment->id(), segment).second);
}

void BasicIdIndex::AddJunction(const Junction* junction) {
  MALIPUT_THROW_UNLESS(junction_map_.emplace(junction->id(), junction).second);
}

void BasicIdIndex::AddBranchPoint(const BranchPoint* branch_point) {
  MALIPUT_THROW_UNLESS(branch_point_map_.emplace(branch_point->id(), branch_point).second);
}

void BasicIdIndex::WalkAndAddAll(const RoadGeometry* road_geometry) {
  for (int ji = 0; ji < road_geometry->num_junctions(); ++ji) {
    const Junction* junction = road_geometry->junction(ji);
    AddJunction(junction);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const Segment* segment = junction->segment(si);
      AddSegment(segment);
      for (int li = 0; li < segment->num_lanes(); ++li) {
        AddLane(segment->lane(li));
      }
    }
  }
  for (int bi = 0; bi < road_geometry->num_branch_points(); ++bi) {
    AddBranchPoint(road_geometry->branch_point(bi));
  }
}

namespace {
template <typename T, typename U>
T find_or_nullptr(const std::unordered_map<U, T>& map, const U& id) {
  auto it = map.find(id);
  return (it == map.end()) ? nullptr : it->second;
}
}  // namespace

const Lane* BasicIdIndex::DoGetLane(const LaneId& id) const { return find_or_nullptr(lane_map_, id); }

const std::unordered_map<LaneId, const Lane*>& BasicIdIndex::DoGetLanes() const { return lane_map_; }

const Segment* BasicIdIndex::DoGetSegment(const SegmentId& id) const { return find_or_nullptr(segment_map_, id); }

const Junction* BasicIdIndex::DoGetJunction(const JunctionId& id) const { return find_or_nullptr(junction_map_, id); }

const BranchPoint* BasicIdIndex::DoGetBranchPoint(const BranchPointId& id) const {
  return find_or_nullptr(branch_point_map_, id);
}

}  // namespace api
}  // namespace maliput
