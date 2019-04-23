#pragma once

#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the mapping from RightOfWayRule::Id to
/// PhaseRing.
class PhaseRingBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseRingBook);

  virtual ~PhaseRingBook() = default;

  /// Gets the specified PhaseRing. Returns drake::nullopt if @p ring_id is
  /// unrecognized.
  drake::optional<PhaseRing> GetPhaseRing(const PhaseRing::Id& ring_id) const {
    return DoGetPhaseRing(ring_id);
  }

  /// Finds and returns the PhaseRing containing the specified
  /// RightOfWayRule. Returns drake::nullopt if @p rule_id is unrecognized.
  drake::optional<PhaseRing> FindPhaseRing(const RightOfWayRule::Id& rule_id) const {
    return DoFindPhaseRing(rule_id);
  }

 protected:
  PhaseRingBook() = default;

 private:
  virtual drake::optional<PhaseRing> DoGetPhaseRing(const PhaseRing::Id& ring_id)
      const = 0;

  virtual drake::optional<PhaseRing> DoFindPhaseRing(const RightOfWayRule::Id& rule_id)
      const  = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
