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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

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
