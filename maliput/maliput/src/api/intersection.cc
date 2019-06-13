#include "maliput/api/intersection.h"

namespace maliput {
namespace api {

Intersection::Intersection(const Id& id,
                           const std::vector<rules::LaneSRange>& region,
                           const rules::PhaseRing::Id& ring_id,
                           const rules::PhaseRingBook* phase_ring_book)
    : id_(id), region_(region), ring_id_(ring_id),
      phase_ring_book_(phase_ring_book) {}

const drake::optional<rules::BulbStates>& Intersection::bulb_states() const {
	drake::optional<rules::PhaseProvider::Result> phase_result = Phase();
	if (phase_result) {
		const Phase::Id phase_id = phase.;
	}
	return drake::nullopt;
}

}  // namespace api
}  // namespace maliput
