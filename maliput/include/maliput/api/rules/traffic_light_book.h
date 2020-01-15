#pragma once

#include <optional>
#include <vector>

#include "maliput/api/rules/traffic_lights.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the mapping from TrafficLight::Id to
/// TrafficLight.
class TrafficLightBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficLightBook);

  virtual ~TrafficLightBook() = default;

  /// Returns all TrafficLights in this book.
  std::vector<const TrafficLight*> TrafficLights() const { return DoTrafficLights(); }

  /// Gets the specified TrafficLight. Returns nullptr if @p id is unrecognized.
  const TrafficLight* GetTrafficLight(const TrafficLight::Id& id) const { return DoGetTrafficLight(id); }

 protected:
  TrafficLightBook() = default;

 private:
  virtual const TrafficLight* DoGetTrafficLight(const TrafficLight::Id& id) const = 0;

  virtual std::vector<const TrafficLight*> DoTrafficLights() const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
