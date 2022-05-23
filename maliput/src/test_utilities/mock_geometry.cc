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
#include "maliput/test_utilities/mock_geometry.h"

#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace geometry_base {
namespace test {

api::RoadPositionResult MockRoadGeometry::DoToRoadPosition(const api::InertialPosition&,
                                                           const std::optional<api::RoadPosition>&) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

std::vector<api::RoadPositionResult> MockRoadGeometry::DoFindRoadPositions(const api::InertialPosition&, double) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

double MockLane::do_length() const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::RBounds MockLane::do_lane_bounds(double) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::RBounds MockLane::do_segment_bounds(double) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::HBounds MockLane::do_elevation_bounds(double, double) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::InertialPosition MockLane::DoToInertialPosition(const api::LanePosition&) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::Rotation MockLane::DoGetOrientation(const api::LanePosition&) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::LanePosition MockLane::DoEvalMotionDerivatives(const api::LanePosition&, const api::IsoLaneVelocity&) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

api::LanePositionResult MockLane::DoToLanePosition(const api::InertialPosition&) const {
  MALIPUT_THROW_UNLESS(false);
  MALIPUT_ABORT_MESSAGE("Not implemented.");
}

}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
