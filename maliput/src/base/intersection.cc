#include "maliput/base/intersection.h"

#include "maliput/common/maliput_throw.h"

namespace maliput {

Intersection::Intersection(const Id& id, const std::vector<api::LaneSRange>& region, const api::rules::PhaseRing& ring,
                           ManualPhaseProvider* phase_provider)
    : api::Intersection(id, region, ring), phase_provider_(phase_provider) {
  MALIPUT_THROW_UNLESS(phase_provider_ != nullptr);
}

std::optional<api::rules::PhaseProvider::Result> Intersection::Phase() const {
  return phase_provider_->GetPhase(ring_id());
}

void Intersection::SetPhase(const api::rules::Phase::Id& phase_id,
                            const std::optional<api::rules::Phase::Id>& next_phase,
                            const std::optional<double>& duration_until) {
  phase_provider_->SetPhase(ring_id(), phase_id, next_phase, duration_until);
}

}  // namespace maliput
