// BSD 3-Clause License
//
// Copyright (c) 2026, Woven by Toyota. All rights reserved.
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

#include "maliput/api/lane.h"
#include "maliput/api/objects/road_marking.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace objects {

/// Abstract interface for providing the mapping from RoadMarking::Id to
/// RoadMarking.
///
/// This follows the same pattern as RoadObjectBook and TrafficSignBook.
/// Backend implementations are responsible for populating the book with the
/// road markings present in their road network data sources.
class RoadMarkingBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadMarkingBook)

  virtual ~RoadMarkingBook() = default;

  /// Returns all RoadMarkings in this book.
  std::vector<const RoadMarking*> RoadMarkings() const { return DoRoadMarkings(); }

  /// Gets the specified RoadMarking. Returns nullptr if @p id is unrecognized.
  const RoadMarking* GetRoadMarking(const RoadMarking::Id& id) const { return DoGetRoadMarking(id); }

  /// Returns all RoadMarkings whose related_lanes() includes @p lane_id.
  ///
  /// Returns an empty vector if no markings are associated with the given lane.
  std::vector<const RoadMarking*> FindByLane(const LaneId& lane_id) const { return DoFindByLane(lane_id); }

  /// Returns all RoadMarkings whose type matches @p type.
  ///
  /// Returns an empty vector if no markings match.
  std::vector<const RoadMarking*> FindByType(RoadMarkingType type) const { return DoFindByType(type); }

 protected:
  RoadMarkingBook() = default;

 private:
  virtual std::vector<const RoadMarking*> DoRoadMarkings() const = 0;

  virtual const RoadMarking* DoGetRoadMarking(const RoadMarking::Id& id) const = 0;

  virtual std::vector<const RoadMarking*> DoFindByLane(const LaneId& lane_id) const = 0;

  virtual std::vector<const RoadMarking*> DoFindByType(RoadMarkingType type) const = 0;
};

}  // namespace objects
}  // namespace api
}  // namespace maliput
