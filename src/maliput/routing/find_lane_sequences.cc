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

#include <algorithm>

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

// Convenient struct to hold both incoming and ongoing LaneEnds.
// It will be used in HasUTurn to identify when a sequence of Lanes implies a U-turn.
struct ConnectionSet {
  LaneEnd incoming;
  LaneEnd ongoing;
};

// Builds a ConnectionSet from two adjancent Lanes.
//
// The method will look for the LaneEnd::kStart end of @p incoming_lane first for ongoing_lane.
// Then, it will look for @p ongoing_lane in the LaneEnd::kFinish end. This sets a precendence
// order in case of two Lanes that are connected in both ends, typically building a roundabout
// or similar geometry. This is a known behavior from FindLaneSequences() which can work with
// roundabouts, given that the same Lane cannot be twice in the sequence.
//
// @param incoming_lane The incoming Lane. It must not be nullptr.
// @param ongoing_lane The ongoing Lane. It must not be nullptr.
// @return A ConnectionSet.
// @throws common::assertion_error When @p incoming_lane or @p ongoing_lane are nullptr.
// @throws common::assertion_error When no ConnectionSet could be built.
ConnectionSet BuildConnectionSet(const Lane* incoming_lane, const Lane* ongoing_lane) {
  MALIPUT_THROW_UNLESS(incoming_lane != nullptr);
  MALIPUT_THROW_UNLESS(ongoing_lane != nullptr);
  // Analize the start end.
  const LaneEndSet* start_ongoing_branches = incoming_lane->GetOngoingBranches(LaneEnd::kStart);
  for (int i = 0; i < start_ongoing_branches->size(); ++i) {
    const LaneEnd ongoing = start_ongoing_branches->get(i);
    if (ongoing.lane == ongoing_lane) {
      return ConnectionSet{LaneEnd(incoming_lane, LaneEnd::kStart), ongoing};
    }
  }
  // Analize the finish end.
  const LaneEndSet* finish_ongoing_branches = incoming_lane->GetOngoingBranches(LaneEnd::kFinish);
  for (int i = 0; i < finish_ongoing_branches->size(); ++i) {
    const LaneEnd ongoing = finish_ongoing_branches->get(i);
    if (ongoing.lane == ongoing_lane) {
      return ConnectionSet{LaneEnd(incoming_lane, LaneEnd::kFinish), ongoing};
    }
  }
  // It was impossible to find a connection between the two lanes, an error has occured somewhere else.
  MALIPUT_THROW_MESSAGE("maliput::routing::BuildConnectionSet(): code must not reach here.");
}

// Determines whether @p lane_sequence has a U-turn by analyzing the LaneEnd sequences.
// @param lane_sequence The sequence of Lanes to analyze for U-turns. It must not be empty.
// @return true When @p lane_sequence has a U-turn.
// @throws common::assertion_error When @p lane_sequence is empty.
bool HasUTurn(const std::vector<const Lane*>& lane_sequence) {
  // Analize preconditions.
  MALIPUT_THROW_UNLESS(!lane_sequence.empty());
  // Escape condition: when lane_sequence has one or two elements, we have no U turn.
  if (lane_sequence.size() <= 2u) {
    return false;
  }
  // Obtain the first connection_set.
  ConnectionSet connection_set = BuildConnectionSet(lane_sequence[0], lane_sequence[1]);
  for (size_t i = 1; i < lane_sequence.size() - 1u; ++i) {
    // Obtain the following connection_set.
    const ConnectionSet next_connection_set = BuildConnectionSet(lane_sequence[i], lane_sequence[i + 1]);
    // Evaluate if there is a U turn.
    if (next_connection_set.ongoing.end == connection_set.incoming.end) {
      return true;
    }
    connection_set = next_connection_set;
  }
  return false;
}

/// Remove sequences with U-turns from @p lane_sequences.
/// @param lane_sequences A sequence of Lanes. It is expected to be the result of FindLaneSequences.
/// @return A filtered copy of @p lane_sequences without those sequences that present a U-turn.
std::vector<std::vector<const Lane*>> RemoveUTurns(const std::vector<std::vector<const Lane*>>& lane_sequences) {
  std::vector<std::vector<const Lane*>> result;
  std::copy_if(lane_sequences.begin(), lane_sequences.end(), std::back_inserter(result), std::not_fn(HasUTurn));
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

std::vector<std::vector<const Lane*>> FindLaneSequences(const Lane* start, const Lane* end, double max_length_m,
                                                        bool no_u_turns) {
  MALIPUT_PROFILE_FUNC();
  const std::vector<std::vector<const Lane*>> unfiltered_result = FindLaneSequences(start, end, max_length_m);
  return no_u_turns ? RemoveUTurns(unfiltered_result) : unfiltered_result;
}

}  // namespace routing
}  // namespace maliput
