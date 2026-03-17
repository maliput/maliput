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
#include "maliput/base/road_object_book.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "maliput/common/maliput_throw.h"

namespace maliput {

using api::InertialPosition;
using api::LaneId;
using api::objects::RoadObject;
using api::objects::RoadObjectType;

class RoadObjectBook::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() = default;
  ~Impl() = default;

  void AddRoadObject(std::unique_ptr<RoadObject> road_object) {
    MALIPUT_THROW_UNLESS(road_object.get() != nullptr);
    const RoadObject::Id id = road_object->id();
    auto result = book_.emplace(id, std::move(road_object));
    if (!result.second) {
      throw std::logic_error("Attempted to add multiple RoadObject instances with ID: " + id.string());
    }
  }

  std::vector<const RoadObject*> DoRoadObjects() const {
    std::vector<const RoadObject*> result;
    result.reserve(book_.size());
    std::transform(book_.begin(), book_.end(), std::back_inserter(result),
                   [](const auto& key_value) { return key_value.second.get(); });
    return result;
  }

  const RoadObject* DoGetRoadObject(const RoadObject::Id& id) const {
    auto it = book_.find(id);
    return it == book_.end() ? nullptr : it->second.get();
  }

  std::vector<const RoadObject*> DoFindByType(RoadObjectType type) const {
    std::vector<const RoadObject*> result;
    for (const auto& [obj_id, obj] : book_) {
      if (obj->type() == type) {
        result.push_back(obj.get());
      }
    }
    return result;
  }

  std::vector<const RoadObject*> DoFindByLane(const LaneId& lane_id) const {
    std::vector<const RoadObject*> result;
    for (const auto& [obj_id, obj] : book_) {
      const auto& lanes = obj->related_lanes();
      if (std::find(lanes.begin(), lanes.end(), lane_id) != lanes.end()) {
        result.push_back(obj.get());
      }
    }
    return result;
  }

  std::vector<const RoadObject*> DoFindInRadius(const InertialPosition& position, double radius) const {
    MALIPUT_THROW_UNLESS(radius >= 0.);
    const double radius_sq = radius * radius;
    std::vector<const RoadObject*> result;
    for (const auto& [obj_id, obj] : book_) {
      const auto& obj_pos = obj->position().inertial_position();
      const double dx = obj_pos.x() - position.x();
      const double dy = obj_pos.y() - position.y();
      const double dz = obj_pos.z() - position.z();
      if (dx * dx + dy * dy + dz * dz <= radius_sq) {
        result.push_back(obj.get());
      }
    }
    return result;
  }

 private:
  std::unordered_map<RoadObject::Id, std::unique_ptr<RoadObject>> book_;
};

RoadObjectBook::RoadObjectBook() : impl_(std::make_unique<Impl>()) {}

RoadObjectBook::~RoadObjectBook() = default;

void RoadObjectBook::AddRoadObject(std::unique_ptr<RoadObject> road_object) {
  impl_->AddRoadObject(std::move(road_object));
}

std::vector<const RoadObject*> RoadObjectBook::DoRoadObjects() const { return impl_->DoRoadObjects(); }

const RoadObject* RoadObjectBook::DoGetRoadObject(const RoadObject::Id& id) const { return impl_->DoGetRoadObject(id); }

std::vector<const RoadObject*> RoadObjectBook::DoFindByType(RoadObjectType type) const {
  return impl_->DoFindByType(type);
}

std::vector<const RoadObject*> RoadObjectBook::DoFindByLane(const LaneId& lane_id) const {
  return impl_->DoFindByLane(lane_id);
}

std::vector<const RoadObject*> RoadObjectBook::DoFindInRadius(const InertialPosition& position, double radius) const {
  return impl_->DoFindInRadius(position, radius);
}

}  // namespace maliput
