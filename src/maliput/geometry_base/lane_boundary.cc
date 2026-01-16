// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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
#include "maliput/geometry_base/lane_boundary.h"

#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/geometry_base/branch_point.h"

namespace maliput {
namespace geometry_base {

void LaneBoundary::AttachToSegment(common::Passkey<Segment>, const api::Segment* segment, int index) {
  // Parameter checks
  MALIPUT_THROW_UNLESS(segment != nullptr);
  MALIPUT_THROW_UNLESS(index >= 0);
  // Preconditions
  MALIPUT_THROW_UNLESS(segment_ == nullptr);
  MALIPUT_THROW_UNLESS(index_ == -1);

  segment_ = segment;
  index_ = index;
}

std::optional<api::LaneMarkingResult> LaneBoundary::DoGetMarking(double s) const {
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

std::vector<api::LaneMarkingResult> LaneBoundary::DoGetMarkings() const {
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

std::vector<api::LaneMarkingResult> LaneBoundary::DoGetMarkings(double s_start, double s_end) const {
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

}  // namespace geometry_base
}  // namespace maliput
