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
#pragma once

#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"

namespace maliput {
namespace geometry_base {

/// Provides a brute force implementation of RoadGeometry::FindRoadPosition()
/// that exhaustively calls `ToLanePosition()` on all lanes and checks
/// _distance_ with `radius`.
///
/// @note Maliput backends could avoid implementing a custom implementation of
/// RoadGeometry::FindRoadPositions() that knows about the geometry internals by
/// forwarding calls to this function. On the contrary, backends might decide
/// not to use this function because, for example, time complexity which is at
/// least O(n^3).
///
/// @param rg The RoadGeometry over all these operations are performed. It
///        must not be nullptr.
/// @param inertial_position The inertial position to convert into one or more
///        RoadPositions.
/// @param radius The maximum distance from @p inertial_position to search. It
///        must not be negative.
/// @return A vector of RoadPositionResults representing the possible
///         RoadPositions.
/// @throws maliput::common::assertion_error If rg is nullptr, or any entity
///         within it is nullptr.
/// @throws maliput::common::assertion_error If radius is negative.
std::vector<maliput::api::RoadPositionResult> BruteForceFindRoadPositionsStrategy(
    const maliput::api::RoadGeometry* rg, const maliput::api::InertialPosition& inertial_position, double radius);

}  // namespace geometry_base
}  // namespace maliput
