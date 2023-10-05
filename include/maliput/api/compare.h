// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota.
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
#pragma once

#include "maliput/api/lane_data.h"
#include "maliput/common/compare.h"

namespace maliput {
namespace api {

/// Compares equality within @p tolerance deviation of the maliput::api::LanePositionResult @p lpr_a and @p lpr_B.
/// @param lpr_a The first LanePositionResult to compare.
/// @param lpr_b The second LanePositionResult to compare.
/// @param tolerance The tolerance to use for the comparison.
/// @returns A ComparisonResult indicating whether the two LanePositionResults are close.
common::ComparisonResult<LanePositionResult> IsLanePositionResultClose(const LanePositionResult& lpr_a,
                                                                       const LanePositionResult& lpr_b,
                                                                       double tolerance);

/// Compares equality within @p tolerance deviation of the maliput::api::RoadPositionResult @p rpr_a and @p rpr_B.
/// @param rpr_a The first RoadPositionResult to compare.
/// @param rpr_b The second RoadPositionResult to compare.
/// @param tolerance The tolerance to use for the comparison.
/// @returns A ComparisonResult indicating whether the two RoadPositionResults are close.
common::ComparisonResult<RoadPositionResult> IsRoadPositionResultClose(const maliput::api::RoadPositionResult& rpr_a,
                                                                       const maliput::api::RoadPositionResult& rpr_b,
                                                                       double tolerance);

/// Compares equality within @p tolerance deviation of two InertialPosition objects.
/// @param pos1 A InertialPosition object to compare.
/// @param pos2 A InertialPosition object to compare.
/// @param tolerance An allowable absolute deviation for each InertialPosition's
/// coordinate.
/// @return A ComparisonResult indicating whether the two InertialPositions are close.
common::ComparisonResult<InertialPosition> IsInertialPositionClose(const InertialPosition& pos1,
                                                                   const InertialPosition& pos2, double tolerance);

/// Compares equality within @p tolerance deviation of two LanePosition objects.
/// @param pos1 A LanePosition object to compare.
/// @param pos2 A LanePosition object to compare.
/// @param tolerance An allowable absolute deviation for each LanePosition's
/// coordinate.
/// @returns A ComparisonResult indicating whether the two LanePositions are close.
common::ComparisonResult<LanePosition> IsLanePositionClose(const LanePosition& pos1, const LanePosition& pos2,
                                                           double tolerance);

/// Compares equality within @p tolerance deviation of two Rotation objects.
/// Comparison will evaluate the inner Rotation's Euler angles.
/// @param rot1 A Rotation object to compare.
/// @param rot2 A Rotation object to compare.
/// @param tolerance An allowable absolute deviation for each Rotation's
/// coordinate.
/// @returns A ComparisonResult indicating whether the two Rotations are close.
common::ComparisonResult<Rotation> IsRotationClose(const Rotation& rot1, const Rotation& rot2, double tolerance);

/// Compares equality within @p tolerance deviation of two RBounds objects.
/// @param rbounds1 A RBounds object to compare.
/// @param rbounds2 A RBounds object to compare.
/// @param tolerance An allowable absolute deviation for each RBounds's instance
/// value.
/// @returns A ComparisonResult indicating whether the two RBounds are close.
common::ComparisonResult<RBounds> IsRBoundsClose(const RBounds& rbounds1, const RBounds& rbounds2, double tolerance);

/// Compares equality within @p tolerance deviation of two HBounds objects.
/// @param hbounds1 A HBounds object to compare.
/// @param hbounds1 A HBounds object to compare.
/// @param tolerance An allowable absolute deviation for each HBounds's instance
/// value.
/// @returns A ComparisonResult indicating whether the two HBounds are close.
common::ComparisonResult<HBounds> IsHBoundsClose(const HBounds& hbounds1, const HBounds& hbounds2, double tolerance);

/// Compares equality of two LaneEnd objects.
/// @param lane_end1 A LaneEnd object to compare.
/// @param lane_end2 A LaneEnd object to compare.
/// @returns A ComparisonResult indicating whether the two LaneEnds are equal.
common::ComparisonResult<LaneEnd> IsLaneEndEqual(const LaneEnd& lane_end1, const LaneEnd& lane_end2);

}  // namespace api
}  // namespace maliput
