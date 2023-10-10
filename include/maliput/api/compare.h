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

#include <optional>
#include <string>

#include "maliput/api/branch_point.h"
#include "maliput/api/junction.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"
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

/// Compares equality of two Junction objects.
/// @param a_expression Literal expression of the first Junction.
/// @param b_expression Literal expression of the second Junction.
/// @param a The first Junction to compare.
/// @param b The second Junction to compare.
/// @returns A ComparisonResult indicating whether the two Junctions are the same.
common::ComparisonResult<Junction> IsEqual(const char* a_expression, const char* b_expression, const Junction* a,
                                           const Junction* b);

/// Compares equality of two Segment objects.
/// @param a_expression Literal expression of the first Segment.
/// @param b_expression Literal expression of the second Segment.
/// @param a The first Segment to compare.
/// @param b The second Segment to compare.
/// @returns A ComparisonResult indicating whether the two Segments are the same.
common::ComparisonResult<Segment> IsEqual(const char* a_expression, const char* b_expression, const Segment* a,
                                          const Segment* b);

/// Compares equality of two Lane objects.
/// @param a_expression Literal expression of the first Lane.
/// @param b_expression Literal expression of the second Lane.
/// @param a The first Lane to compare.
/// @param b The second Lane to compare.
/// @returns A ComparisonResult indicating whether the two Lanes are the same.
common::ComparisonResult<Lane> IsEqual(const char* a_expression, const char* b_expression, const Lane* a,
                                       const Lane* b);

/// Compares equality of two BranchPoint objects.
/// @param a_expression Literal expression of the first BranchPoint.
/// @param b_expression Literal expression of the second BranchPoint.
/// @param a The first BranchPoint to compare.
/// @param b The second BranchPoint to compare.
/// @returns A ComparisonResult indicating whether the two BranchPoints are the same.
common::ComparisonResult<BranchPoint> IsEqual(const char* a_expression, const char* b_expression, const BranchPoint* a,
                                              const BranchPoint* b);

/// Compares equality of two booleans.
/// @param a_expression Literal expression of the first boolean.
/// @param b_expression Literal expression of the second boolean.
/// @param a The first boolean to compare.
/// @param b The second boolean to compare.
/// @returns A ComparisonResult indicating whether the two booleans are equal.
common::ComparisonResult<bool> IsEqual(const char* a_expression, const char* b_expression, bool a, bool b);

/// Compares equality of two doubles.
/// @param a_expression Literal expression of the first double.
/// @param b_expression Literal expression of the second double.
/// @param a The first double to compare.
/// @param b The second double to compare.
/// @returns A ComparisonResult indicating whether the two doubles are equal.
common::ComparisonResult<double> IsEqual(const char* a_expression, const char* b_expression, double a, double b);

/// Compares equality of two std::size_t.
/// @param a_expression Literal expression of the first std::size_t.
/// @param b_expression Literal expression of the second std::size_t.
/// @param a The first std::size_t to compare.
/// @param b The second std::size_t to compare.
/// @returns A ComparisonResult indicating whether the two std::size_t are equal.
common::ComparisonResult<std::size_t> IsEqual(const char* a_expression, const char* b_expression, std::size_t a,
                                              std::size_t b);

/// Compares equality of two TypeSpecificIdentifier<T>.
/// @paramT T The type of the TypeSpecificIdentifier.
/// @param a_expression Literal expression of the first TypeSpecificIdentifier.
/// @param b_expression Literal expression of the second TypeSpecificIdentifier.
/// @param a The first TypeSpecificIdentifier to compare.
/// @param b The second TypeSpecificIdentifier to compare.
/// @returns A ComparisonResult indicating whether the two TypeSpecificIdentifiers are equal.
template <typename T>
common::ComparisonResult<TypeSpecificIdentifier<T>> IsEqual(const char* a_expression, const char* b_expression,
                                                            const TypeSpecificIdentifier<T>& a,
                                                            const TypeSpecificIdentifier<T>& b) {
  if (a != b) {
    return {"Values are different. " + std::string(a_expression) + ": " + a.string() + " vs. " +
            std::string(b_expression) + ": " + b.string() + "\n"};
  }
  return {std::nullopt};
}

/// Compares equality of two SRanges.
/// @param s_range_1 The first SRange to compare.
/// @param s_range_2 The second SRange to compare.
/// @returns A ComparisonResult indicating whether the two SRanges are equal.
common::ComparisonResult<SRange> IsEqual(const SRange& s_range_1, const SRange& s_range_2);

/// Compares equality of two LaneSRange.
/// @param lane_s_range_1 The first LaneSRange to compare.
/// @param lane_s_range_2 The second LaneSRange to compare.
/// @returns A ComparisonResult indicating whether the two LaneSRanges are equal.
common::ComparisonResult<LaneSRange> IsEqual(const LaneSRange& lane_s_range_1, const LaneSRange& lane_s_range_2);

/// Compares equality of two std::vector<LaneSRange>.
/// @param lane_s_ranges_1 The first std::vector<LaneSRange> to compare.
/// @param lane_s_ranges_2 The second std::vector<LaneSRange> to compare.
/// @return A ComparisonResult indicating whether the two std::vector<LaneSRange> are equal.
common::ComparisonResult<std::vector<LaneSRange>> IsEqual(const std::vector<LaneSRange>& lane_s_ranges_1,
                                                          const std::vector<LaneSRange>& lane_s_ranges_2);

/// Compares equality of two LaneSRoute.
/// @param lane_s_route_1 The first LaneSRoute to compare.
/// @param lane_s_route_2 The second LaneSRoute to compare.
/// @returns A ComparisonResult indicating whether the two LaneSRoutes are equal.
common::ComparisonResult<LaneSRoute> IsEqual(const LaneSRoute& lane_s_route_1, const LaneSRoute& lane_s_route_2);

/// Compares equality of two InertialPosition.
/// @param inertial_position_1 The first InertialPosition to compare.
/// @param inertial_position_2 The second InertialPosition to compare.
/// @returns A ComparisonResult indicating whether the two InertialPositions are equal.
common::ComparisonResult<InertialPosition> IsEqual(const InertialPosition& inertial_position_1,
                                                   const InertialPosition& inertial_position_2);

/// Compares equality of two Rotation.
/// @param rotation_1 The first Rotation to compare.
/// @param rotation_2 The second Rotation to compare.
/// @returns A ComparisonResult indicating whether the two Rotations are equal.
common::ComparisonResult<Rotation> IsEqual(const Rotation& rotation_1, const Rotation& rotation_2);

/// Checks that the given RoadGeometry's ById() indexing is correct.
/// @param road_geometry The RoadGeometry to check.
/// @returns A std::optional<std::string> containing an error message if the indexing is incorrect.
std::optional<std::string> CheckIdIndexing(const RoadGeometry* road_geometry);

}  // namespace api
}  // namespace maliput
