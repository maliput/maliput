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
#include "maliput/routing/find_lane_sequences.h"

#include "maliput/api/branch_point.h"
#include "maliput/api/lane_data.h"
#include "maliput/common/profiler.h"

using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LaneEndSet;

namespace maliput {
namespace routing {
namespace {

// Adds the lanes in @p lane_end_set to @p container.
void AddLanes(const LaneEndSet* lane_end_set, std::vector<const Lane*>* container) {
  for (int i = 0; i < lane_end_set->size(); ++i) {
    const LaneEnd& lane_end = lane_end_set->get(i);
    container->push_back(lane_end.lane);
  }
}

std::vector<std::vector<const Lane*>> FindLaneSequencesHelper(const Lane* start, const Lane* end,
                                                              const std::vector<const Lane*>& visited_list,
                                                              double max_length_m, double current_length_m) {
  std::vector<std::vector<const Lane*>> result;
  if (current_length_m > max_length_m) return result;

  // Gather all of the start lane's ongoing lanes. These are the lanes that will
  // be checked in search for the end lane.
  std::vector<const Lane*> lanes_to_check;
  AddLanes(start->GetOngoingBranches(LaneEnd::kStart), &lanes_to_check);
  AddLanes(start->GetOngoingBranches(LaneEnd::kFinish), &lanes_to_check);

  for (const auto& next_lane : lanes_to_check) {
    if (std::find(visited_list.begin(), visited_list.end(), next_lane) != visited_list.end()) {
      continue;
    }
    if (next_lane->id() == end->id()) {
      result.push_back({start, end});
    } else {
      std::vector<const Lane*> new_visited_list = visited_list;
      new_visited_list.push_back(next_lane);
      const auto subsequences = FindLaneSequencesHelper(next_lane, end, new_visited_list, max_length_m,
                                                        current_length_m + next_lane->length());
      for (std::vector<const Lane*> subsequence : subsequences) {
        subsequence.insert(subsequence.begin(), start);
        result.push_back(subsequence);
      }
    }
  }
  return result;
}

}  // namespace

std::vector<std::vector<const Lane*>> FindLaneSequences(const Lane* start, const Lane* end, double max_length_m) {
  MALIPUT_PROFILE_FUNC();
  if (start->id() == end->id()) {
    return {{start}};
  }
  return FindLaneSequencesHelper(start, end, {start}, max_length_m, 0);
}

}  // namespace routing
}  // namespace maliput
