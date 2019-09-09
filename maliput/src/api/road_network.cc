#include "maliput/api/road_network.h"

#include <utility>

#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {

RoadNetwork::RoadNetwork(std::unique_ptr<const RoadGeometry> road_geometry,
                         std::unique_ptr<const rules::RoadRulebook> rulebook,
                         std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
                         std::unique_ptr<IntersectionBook> intersection_book,
                         std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
                         std::unique_ptr<rules::RightOfWayRuleStateProvider> right_of_way_rule_state_provider,
                         std::unique_ptr<rules::PhaseProvider> phase_provider)
    : road_geometry_(std::move(road_geometry)),
      rulebook_(std::move(rulebook)),
      traffic_light_book_(std::move(traffic_light_book)),
      intersection_book_(std::move(intersection_book)),
      phase_ring_book_(std::move(phase_ring_book)),
      right_of_way_rule_state_provider_(std::move(right_of_way_rule_state_provider)),
      phase_provider_(std::move(phase_provider)) {
  MALIPUT_THROW_UNLESS(road_geometry_.get() != nullptr);
  MALIPUT_THROW_UNLESS(rulebook_.get() != nullptr);
  MALIPUT_THROW_UNLESS(traffic_light_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(intersection_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(phase_ring_book_.get() != nullptr);
  MALIPUT_THROW_UNLESS(right_of_way_rule_state_provider_.get() != nullptr);
  MALIPUT_THROW_UNLESS(phase_provider_.get() != nullptr);
}

}  // namespace api
}  // namespace maliput
