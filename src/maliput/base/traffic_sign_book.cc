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
#include "maliput/base/traffic_sign_book.h"

#include <algorithm>
#include <iterator>
#include <string>
#include <unordered_map>
#include <utility>

namespace maliput {

using api::rules::TrafficSign;
using api::rules::TrafficSignType;

class TrafficSignBook::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() = default;
  ~Impl() = default;

  void AddTrafficSign(std::unique_ptr<const TrafficSign> traffic_sign) {
    MALIPUT_THROW_UNLESS(traffic_sign.get() != nullptr);
    const TrafficSign::Id id = traffic_sign->id();
    auto result = book_.emplace(id, std::move(traffic_sign));
    if (!result.second) {
      MALIPUT_THROW_MESSAGE("Attempted to add multiple TrafficSign instances with ID: " + id.string(), maliput::common::traffic_sign_book_error);
    }
  }

  std::vector<const TrafficSign*> DoTrafficSigns() const {
    std::vector<const TrafficSign*> result;
    std::transform(book_.begin(), book_.end(), std::back_inserter(result),
                   [](const auto& key_value) { return key_value.second.get(); });
    return result;
  }

  const TrafficSign* DoGetTrafficSign(const TrafficSign::Id& id) const {
    auto it = book_.find(id);
    return it == book_.end() ? nullptr : it->second.get();
  }

  std::vector<const TrafficSign*> DoFindByLane(const api::LaneId& lane_id) const {
    std::vector<const TrafficSign*> result;
    for (const auto& key_value : book_) {
      const auto& related = key_value.second->related_lanes();
      if (std::find(related.begin(), related.end(), lane_id) != related.end()) {
        result.push_back(key_value.second.get());
      }
    }
    return result;
  }

  std::vector<const TrafficSign*> DoFindByType(const TrafficSignType& type) const {
    std::vector<const TrafficSign*> result;
    for (const auto& key_value : book_) {
      if (key_value.second->type() == type) {
        result.push_back(key_value.second.get());
      }
    }
    return result;
  }

 private:
  std::unordered_map<TrafficSign::Id, std::unique_ptr<const TrafficSign>> book_;
};

TrafficSignBook::TrafficSignBook() : impl_(std::make_unique<Impl>()) {}

TrafficSignBook::~TrafficSignBook() = default;

void TrafficSignBook::AddTrafficSign(std::unique_ptr<const TrafficSign> traffic_sign) {
  impl_->AddTrafficSign(std::move(traffic_sign));
}

const TrafficSign* TrafficSignBook::DoGetTrafficSign(const TrafficSign::Id& id) const {
  return impl_->DoGetTrafficSign(id);
}

std::vector<const TrafficSign*> TrafficSignBook::DoTrafficSigns() const { return impl_->DoTrafficSigns(); }

std::vector<const TrafficSign*> TrafficSignBook::DoFindByLane(const api::LaneId& lane_id) const {
  return impl_->DoFindByLane(lane_id);
}

std::vector<const TrafficSign*> TrafficSignBook::DoFindByType(const TrafficSignType& type) const {
  return impl_->DoFindByType(type);
}

}  // namespace maliput
