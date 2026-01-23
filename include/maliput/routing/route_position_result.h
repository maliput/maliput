// BSD 3-Clause License
//
// Copyright (c) 2023-2026, Woven by Toyota. All rights reserved.
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

/// The result of a position query on a Phase.
struct PhasePositionResult {
  /// The index of the api::LaneSRange within the Phase.
  int lane_s_range_index{};
  /// The LANE-Frame position within the LaneSRange referenced by `lane_s_range_index`.
  api::LanePosition lane_position;
  /// The INERTIAL-Frame position of `lane_position`.
  api::InertialPosition inertial_position;
  /// The Euclidean distance between the point used for querying and
  /// `inertial_position`.
  double distance{};
};

/// The result of a position query on a Route.
// TODO(#543): Consider leaving just one type and merging PhasePositionResult with this one.
struct RoutePositionResult {
  /// The index of the Phase in the Route.
  int phase_index{};
  /// The position within the Phase.
  PhasePositionResult phase_position_result;
};

}  // namespace routing
}  // namespace maliput
