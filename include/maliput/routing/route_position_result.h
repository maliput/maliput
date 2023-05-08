// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota. All rights reserved.
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
#pragma once

#include "maliput/api/lane_data.h"

namespace maliput {
namespace routing {

/// Maps a position in the LANE and INERTIAL Frames within a RoutePhase.
struct RoutePhasePositionResult {
  /// The maliput::api::LaneSRange within the RoutePhase where this position
  /// is located.
  maliput::api::LaneSRange lane_s_range;
  /// The LANE-Frame position within the `lane_s_range`.
  maliput::api::LanePosition lane_position{};
  /// The INERTIAL-Frame position of `lane_position`.
  maliput::api::InertialPosition inertial_position{};
  /// The Euclidean distance between the point used for querying and
  /// `inertial_position`.
  double distance{};
};

/// Maps a position in a Route.
struct RoutePositionResult {
  /// The index of the RoutePhase in the Route.
  int route_phase_index{};
  /// The RoutePhasePositionResult where the mapping is evaluated.
  RoutePhasePositionResult position;
};

}  // namespace routing
}  // namespace maliput
