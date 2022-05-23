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

#include <string>

#include "maliput/api/road_geometry.h"

namespace maliput {
namespace utility {

/// Parameters that specify what details about an api::RoadGeometry to print.
struct GenerateStringOptions {
  /// Whether to include a label indicating the type, e.g., "geometry",
  /// "junction", "segment", or "lane".
  bool include_type_labels{false};
  bool include_road_geometry_id{false};
  bool include_junction_ids{false};
  bool include_segment_ids{false};
  bool include_lane_ids{false};
  /// Whether to include lane details like the length of the lane's s-curve and
  /// the geo positions at (s, r, h) coordinates (0, 0, 0) and (s_max, 0, 0).
  bool include_lane_details{false};
};

/// Generates and returns a string containing details about the provided
/// api::RoadGeometry.
///
/// @param road_geometry The api::RoadGeometry.
/// @param options Options that affect the types of information to include in
/// the returned string.
/// @return The generated string.
std::string GenerateString(const api::RoadGeometry& road_geometry, const GenerateStringOptions& options);

}  // namespace utility
}  // namespace maliput
