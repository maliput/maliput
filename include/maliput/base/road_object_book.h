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

#include "maliput/api/objects/road_object.h"
#include "maliput/api/objects/road_object_book.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {

/// A concrete implementation of the api::objects::RoadObjectBook abstract
/// interface. It allows users to add road objects and query them by ID,
/// type, lane association, or proximity.
class RoadObjectBook : public api::objects::RoadObjectBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadObjectBook);

  RoadObjectBook();

  ~RoadObjectBook() override;

  /// Adds @p road_object to this RoadObjectBook.
  ///
  /// @throws std::exception if a RoadObject with the same ID already exists.
  void AddRoadObject(std::unique_ptr<api::objects::RoadObject> road_object);

 private:
  std::vector<const api::objects::RoadObject*> DoRoadObjects() const override;

  const api::objects::RoadObject* DoGetRoadObject(const api::objects::RoadObject::Id& id) const override;

  std::vector<const api::objects::RoadObject*> DoFindByType(api::objects::RoadObjectType type) const override;

  std::vector<const api::objects::RoadObject*> DoFindByLane(const api::LaneId& lane_id) const override;

  std::vector<const api::objects::RoadObject*> DoFindInRadius(const api::InertialPosition& position,
                                                              double radius) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
