#include "maliput/api/intersection.h"

namespace maliput {
namespace api {

using rules::BulbStates;
using rules::PhaseProvider;
using rules::PhaseRing;

Intersection::Intersection(const Id& id, const std::vector<LaneSRange>& region, const PhaseRing& ring)
    : id_(id), region_(region), ring_(ring) {}

const drake::optional<BulbStates> Intersection::bulb_states() const {
  drake::optional<PhaseProvider::Result> phase_result = Phase();
  if (phase_result.has_value()) {
    const rules::Phase::Id phase_id = phase_result->id;
    const rules::Phase& phase = ring_.phases().at(phase_id);
    return phase.bulb_states();
  }
  return drake::nullopt;
}

}  // namespace api
}  // namespace maliput
