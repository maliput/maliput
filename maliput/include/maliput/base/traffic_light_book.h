#pragma once

#include <memory>
#include <vector>
#include <optional>

#include "drake/common/drake_copyable.h"
#include "maliput/api/rules/traffic_light_book.h"
#include "maliput/api/rules/traffic_lights.h"

namespace maliput {

/// A concrete implementation of the api::rules::TrafficLightBook abstract
/// interface. It allows users to obtain a TrafficLight using its ID.
class TrafficLightBook : public api::rules::TrafficLightBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficLightBook);

  TrafficLightBook();

  ~TrafficLightBook() override;

  /// Adds @p traffic_light to this TrafficLightBook.
  ///
  /// @throws std::exception if an api::rules::TrafficLight with the same ID
  /// already exists.
  void AddTrafficLight(std::unique_ptr<const api::rules::TrafficLight> traffic_light);

 private:
  const api::rules::TrafficLight* DoGetTrafficLight(const api::rules::TrafficLight::Id& id) const override;

  std::vector<const api::rules::TrafficLight*> DoTrafficLights() const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace maliput
