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
#include "maliput/geometry_base/road_geometry.h"

#include <utility>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace geometry_base {

void RoadGeometry::AddJunctionPrivate(std::unique_ptr<Junction> junction) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(junction.get() != nullptr);
  junctions_.emplace_back(std::move(junction));
  Junction* const raw_junction = junctions_.back().get();
  // clang-format off
  raw_junction->AttachToRoadGeometry({}, this, [this](auto segment) { id_index_.AddSegment(segment); },
                                     [this](auto lane) { id_index_.AddLane(lane); });
  // clang-format on
  id_index_.AddJunction(raw_junction);
}

void RoadGeometry::AddBranchPointPrivate(std::unique_ptr<BranchPoint> branch_point) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(branch_point.get() != nullptr);
  branch_points_.emplace_back(std::move(branch_point));
  BranchPoint* const raw_branch_point = branch_points_.back().get();
  raw_branch_point->AttachToRoadGeometry({}, this);
  id_index_.AddBranchPoint(raw_branch_point);
}

const api::Junction* RoadGeometry::do_junction(int index) const { return junctions_.at(index).get(); }

const api::BranchPoint* RoadGeometry::do_branch_point(int index) const { return branch_points_.at(index).get(); }

}  // namespace geometry_base
}  // namespace maliput
