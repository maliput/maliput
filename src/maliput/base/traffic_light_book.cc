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
#include "maliput/base/traffic_light_book.h"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace maliput {

using api::rules::TrafficLight;

class TrafficLightBook::Impl {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() = default;
  ~Impl() = default;

  void AddTrafficLight(std::unique_ptr<const TrafficLight> traffic_light) {
    MALIPUT_THROW_UNLESS(traffic_light.get() != nullptr);
    const TrafficLight::Id id = traffic_light->id();
    auto result = book_.emplace(id, std::move(traffic_light));
    if (!result.second) {
      throw std::logic_error("Attempted to add multiple TrafficLight instances with ID: " + id.string());
    }
  }

  std::vector<const TrafficLight*> DoTrafficLights() const {
    std::vector<const TrafficLight*> result;
    std::transform(book_.begin(), book_.end(), std::back_inserter(result),
                   [](const auto& key_value) { return key_value.second.get(); });
    return result;
  }

  const TrafficLight* DoGetTrafficLight(const TrafficLight::Id& id) const {
    auto it = book_.find(id);
    return it == book_.end() ? nullptr : it->second.get();
  }

 private:
  std::unordered_map<TrafficLight::Id, std::unique_ptr<const TrafficLight>> book_;
};

TrafficLightBook::TrafficLightBook() : impl_(std::make_unique<Impl>()) {}

TrafficLightBook::~TrafficLightBook() = default;

void TrafficLightBook::AddTrafficLight(std::unique_ptr<const TrafficLight> traffic_light) {
  impl_->AddTrafficLight(std::move(traffic_light));
}

const TrafficLight* TrafficLightBook::DoGetTrafficLight(const TrafficLight::Id& id) const {
  return impl_->DoGetTrafficLight(id);
}

std::vector<const TrafficLight*> TrafficLightBook::DoTrafficLights() const { return impl_->DoTrafficLights(); }

}  // namespace maliput
