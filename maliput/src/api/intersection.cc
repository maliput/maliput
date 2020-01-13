#include "maliput/api/intersection.h"

namespace maliput {
namespace api {

using rules::BulbStates;
using rules::PhaseProvider;
using rules::PhaseRing;

Intersection::Intersection(const Id& id, const std::vector<LaneSRange>& region, const PhaseRing& ring)
    : id_(id), region_(region), ring_(ring) {}

const std::optional<BulbStates> Intersection::bulb_states() const {
  std::optional<PhaseProvider::Result> phase_result = Phase();
  if (phase_result.has_value()) {
    const rules::Phase::Id phase_id = phase_result->state;
    const rules::Phase& phase = ring_.phases().at(phase_id);
    return phase.bulb_states();
  }
  return std::nullopt;
}

bool Intersection::Includes(const GeoPosition& geo_position, const RoadGeometry* road_geometry) const {
  MALIPUT_THROW_UNLESS(road_geometry != nullptr);
  return IsIncluded(geo_position, region_, road_geometry);
}

}  // namespace api
}  // namespace maliput
