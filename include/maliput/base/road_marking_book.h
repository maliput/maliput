// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include <memory>
#include <vector>

#include "maliput/api/objects/road_marking.h"
#include "maliput/api/objects/road_marking_book.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {

/// A concrete implementation of the api::objects::RoadMarkingBook abstract
/// interface. It allows users to add road markings and query them by ID,
/// type, or lane association.
class RoadMarkingBook : public api::objects::RoadMarkingBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadMarkingBook);

  RoadMarkingBook();

  ~RoadMarkingBook() override;

  /// Adds @p road_marking to this RoadMarkingBook.
  ///
  /// @throws std::exception if a RoadMarking with the same ID already exists.
  void AddRoadMarking(std::unique_ptr<const api::objects::RoadMarking> road_marking);

 private:
  std::vector<const api::objects::RoadMarking*> DoRoadMarkings() const override;

  const api::objects::RoadMarking* DoGetRoadMarking(const api::objects::RoadMarking::Id& id) const override;

  std::vector<const api::objects::RoadMarking*> DoFindByLane(const api::LaneId& lane_id) const override;

  std::vector<const api::objects::RoadMarking*> DoFindByType(api::objects::RoadMarkingType type) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
