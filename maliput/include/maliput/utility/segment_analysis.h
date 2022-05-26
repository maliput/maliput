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

#include <unordered_set>
#include <vector>

#include "maliput/api/road_geometry.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace utility {

/// Finds all Segments connected to @p seed_segment via confluent Lanes.
///
/// This function performs a breadth-first search over the graph of Segments,
/// Lanes, and BranchPoints originating at `seed_segment`.  It does not
/// explore any other elements and does not require these elements to have
/// valid ownership pointers upwards to Junctions or to a RoadGeometry.
///
/// @returns an unordered_set of Segments connected to `seed_segment`,
///          including `seed_segment` itself.
std::unordered_set<const api::Segment*> FindConfluentSegments(const api::Segment* seed_segment);

/// Analyzes how Segments in @p road_geometry are connected via confluency
/// of their Lanes at BranchPoints.
///
/// Two Lanes which are confluent at a BranchPoint necessarily overlap near
/// the BranchPoint, so the Segments which own those Lanes ought to belong
/// to a common Junction. The output of this function is thus a lower-bound
/// for how Segments should be grouped together into Junctions, which can
/// be used for verifying or synthesizing (approximately) the Junction
/// structure.  (This function will not detect Lanes which have overlapping
/// geometries independent of their BranchPoints.)
///
/// @returns the set of
/// <a href="https://en.wikipedia.org/wiki/Connected_component_(graph_theory)">
///          connected components</a> of Segments, as a vector of
///          unordered_sets.  The ordering of the components in the vector is
///          arbitrary.  Each Segment in @p road_geometry shall belong
///          to exactly one component.
std::vector<std::unordered_set<const api::Segment*>> AnalyzeConfluentSegments(const api::RoadGeometry* road_geometry);

}  // namespace utility
}  // namespace maliput
