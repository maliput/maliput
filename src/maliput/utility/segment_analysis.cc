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
#include "maliput/utility/segment_analysis.h"

#include <queue>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"

namespace maliput {
namespace utility {

std::unordered_set<const api::Segment*> FindConfluentSegments(const api::Segment* const seed_segment) {
  // Make an empty workqueue (FIFO).
  std::queue<const api::Segment*> workqueue;
  // Make an empty visited set.
  std::unordered_set<const api::Segment*> visited;

  // Add seed_segment to the workqueue.
  workqueue.push(seed_segment);

  // Work the queue.
  while (!workqueue.empty()) {
    const api::Segment* working_segment = workqueue.front();
    workqueue.pop();

    // Loop over each Lane in the Segment.
    for (int lane_index = 0; lane_index < working_segment->num_lanes(); ++lane_index) {
      const api::Lane* const lane = working_segment->lane(lane_index);
      if (!lane) {
        continue;
      }
      // Loop over each End of the Lane.
      for (const api::LaneEnd::Which end : {api::LaneEnd::kStart, api::LaneEnd::kFinish}) {
        const api::LaneEndSet* const confluent_set = lane->GetConfluentBranches(end);
        if (!confluent_set) {
          continue;
        }
        // Loop over the set of confluent lanes.
        for (int i = 0; i < confluent_set->size(); ++i) {
          // Get confluent segment.
          const api::Segment* confluent_segment = confluent_set->get(i).lane->segment();
          if (!confluent_segment) {
            continue;
          }
          // Is confluent segment in visited set?
          if (visited.find(confluent_segment) == visited.end()) {
            // Not yet:  then push onto end of workqueue, and mark as visited.
            workqueue.push(confluent_segment);
            visited.insert(confluent_segment);
          }
        }
      }
    }
  }
  // Return the visited set, which is the set of all Segments connected to
  // seed_segment (including seed_segment itself).
  return visited;
}

std::vector<std::unordered_set<const api::Segment*>> AnalyzeConfluentSegments(const api::RoadGeometry* road_geometry) {
  std::vector<std::unordered_set<const api::Segment*>> components;
  std::unordered_set<const api::Segment*> visited;
  for (int junction_index = 0; junction_index < road_geometry->num_junctions(); ++junction_index) {
    const api::Junction* const junction = road_geometry->junction(junction_index);
    if (!junction) {
      continue;
    }
    for (int segment_index = 0; segment_index < junction->num_segments(); ++segment_index) {
      const api::Segment* const segment = junction->segment(segment_index);
      // If segment has already been visited, skip it.
      if (visited.count(segment) > 0) {
        continue;
      }
      // Explore segment's connected component.
      components.emplace_back(FindConfluentSegments(segment));
      // Record all segments from the new component as visited.
      visited.insert(components.back().begin(), components.back().end());
    }
  }
  // Return the list of connected components.
  return components;
}

}  // namespace utility
}  // namespace maliput
