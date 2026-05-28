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
#include "maliput/base/road_marking_book.h"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "maliput/common/maliput_throw.h"

namespace maliput {

using api::LaneId;
using api::objects::RoadMarking;
using api::objects::RoadMarkingType;

class RoadMarkingBook::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() = default;
  ~Impl() = default;

  void AddRoadMarking(std::unique_ptr<const RoadMarking> road_marking) {
    MALIPUT_THROW_UNLESS(road_marking.get() != nullptr);
    const RoadMarking::Id id = road_marking->id();
    auto result = book_.emplace(id, std::move(road_marking));
    if (!result.second) {
      throw std::logic_error("Attempted to add multiple RoadMarking instances with ID: " + id.string());
    }
  }

  std::vector<const RoadMarking*> DoRoadMarkings() const {
    std::vector<const RoadMarking*> result;
    result.reserve(book_.size());
    std::transform(book_.begin(), book_.end(), std::back_inserter(result),
                   [](const auto& key_value) { return key_value.second.get(); });
    return result;
  }

  const RoadMarking* DoGetRoadMarking(const RoadMarking::Id& id) const {
    auto it = book_.find(id);
    return it == book_.end() ? nullptr : it->second.get();
  }

  std::vector<const RoadMarking*> DoFindByLane(const LaneId& lane_id) const {
    std::vector<const RoadMarking*> result;
    for (const auto& [marking_id, marking] : book_) {
      const auto& lanes = marking->related_lanes();
      if (std::find(lanes.begin(), lanes.end(), lane_id) != lanes.end()) {
        result.push_back(marking.get());
      }
    }
    return result;
  }

  std::vector<const RoadMarking*> DoFindByType(RoadMarkingType type) const {
    std::vector<const RoadMarking*> result;
    for (const auto& [marking_id, marking] : book_) {
      if (marking->type() == type) {
        result.push_back(marking.get());
      }
    }
    return result;
  }

 private:
  std::unordered_map<RoadMarking::Id, std::unique_ptr<const RoadMarking>> book_;
};

RoadMarkingBook::RoadMarkingBook() : impl_(std::make_unique<Impl>()) {}

RoadMarkingBook::~RoadMarkingBook() = default;

void RoadMarkingBook::AddRoadMarking(std::unique_ptr<const RoadMarking> road_marking) {
  impl_->AddRoadMarking(std::move(road_marking));
}

std::vector<const RoadMarking*> RoadMarkingBook::DoRoadMarkings() const { return impl_->DoRoadMarkings(); }

const RoadMarking* RoadMarkingBook::DoGetRoadMarking(const RoadMarking::Id& id) const {
  return impl_->DoGetRoadMarking(id);
}

std::vector<const RoadMarking*> RoadMarkingBook::DoFindByLane(const LaneId& lane_id) const {
  return impl_->DoFindByLane(lane_id);
}

std::vector<const RoadMarking*> RoadMarkingBook::DoFindByType(RoadMarkingType type) const {
  return impl_->DoFindByType(type);
}

}  // namespace maliput
