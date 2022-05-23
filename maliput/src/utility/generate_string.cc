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
#include "maliput/utility/generate_string.h"

#include <sstream>

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/segment.h"

namespace maliput {
namespace utility {
namespace {

constexpr int kIndentSize{2};  // Spaces.

struct IndentLevels {
  int junction{0};
  int segment{0};
  int lane{0};
};

IndentLevels ComputeIndentLevels(const GenerateStringOptions& options) {
  IndentLevels result;
  if (options.include_segment_ids) {
    ++result.lane;
  }
  if (options.include_junction_ids) {
    ++result.segment;
    ++result.lane;
  }
  if (options.include_road_geometry_id) {
    ++result.lane;
    ++result.segment;
    ++result.junction;
  }
  return result;
}

std::string GetIndent(int indent) { return std::string(indent * kIndentSize, ' '); }

}  // namespace

std::string GenerateString(const api::RoadGeometry& road_geometry, const GenerateStringOptions& options) {
  const IndentLevels indent = ComputeIndentLevels(options);
  std::stringstream result;
  if (options.include_road_geometry_id) {
    if (options.include_type_labels) {
      result << "geometry: ";
    }
    result << road_geometry.id().string() << "\n";
  }

  const std::string junction_prefix = GetIndent(indent.junction);
  for (int ji = 0; ji < road_geometry.num_junctions(); ++ji) {
    const api::Junction* junction = road_geometry.junction(ji);
    if (!junction) {
      continue;
    }
    if (options.include_junction_ids) {
      result << junction_prefix;
      if (options.include_type_labels) {
        result << "junction: ";
      }
      result << junction->id().string() << "\n";
    }
    const std::string segment_prefix = GetIndent(indent.segment);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* segment = junction->segment(si);
      if (options.include_segment_ids) {
        result << segment_prefix;
        if (options.include_type_labels) {
          result << "segment: ";
        }
        result << segment->id().string() << "\n";
      }
      if (options.include_lane_ids) {
        const std::string lane_prefix = GetIndent(indent.lane);
        for (int li = 0; li < segment->num_lanes(); ++li) {
          const api::Lane* lane = segment->lane(li);
          result << lane_prefix;
          if (options.include_type_labels) {
            result << "lane: ";
          }
          result << lane->id().string() << "\n";
          if (options.include_lane_details) {
            result << lane_prefix << "  length: " << lane->length() << "\n";
            result << lane_prefix << "  geo positions:\n";
            result << lane_prefix << "    s_min: " << lane->ToInertialPosition(api::LanePosition(0, 0, 0)) << "\n";
            result << lane_prefix << "    s_max: " << lane->ToInertialPosition(api::LanePosition(lane->length(), 0, 0))
                   << "\n";
            const api::Lane* left = lane->to_left();
            const api::Lane* right = lane->to_right();
            result << lane_prefix << "  to left: " << (left ? left->id().string() : "") << "\n";
            result << lane_prefix << "  to right: " << (right ? right->id().string() : "") << "\n";
          }
        }
      }
    }
  }
  const std::string s = result.str();
  return (s.size() > 0 ? s.substr(0, s.size() - 1) : s);
}

}  // namespace utility
}  // namespace maliput
