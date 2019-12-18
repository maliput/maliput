#pragma once

#include <vector>
#include <optional>

#include "drake/common/drake_copyable.h"

#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/rule.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the mapping from RightOfWayRule::Id to
/// PhaseRing.
class PhaseRingBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseRingBook);

  virtual ~PhaseRingBook() = default;

  /// Gets a list of all PhaseRings within this book.
  std::vector<PhaseRing::Id> GetPhaseRings() const { return DoGetPhaseRings(); }

  /// Gets the specified PhaseRing. Returns std::nullopt if @p ring_id is
  /// unrecognized.
  std::optional<PhaseRing> GetPhaseRing(const PhaseRing::Id& ring_id) const { return DoGetPhaseRing(ring_id); }

  /// Finds and returns the PhaseRing containing the specified
  /// RightOfWayRule. Returns std::nullopt if @p rule_id is unrecognized.
  std::optional<PhaseRing> FindPhaseRing(const RightOfWayRule::Id& rule_id) const { return DoFindPhaseRing(rule_id); }

  /// Finds and returns the PhaseRing containing the specified
  /// Rule. Returns std::nullopt if @p rule_id is unrecognized.
  std::optional<PhaseRing> FindPhaseRing(const Rule::Id& rule_id) const { return DoFindPhaseRing(rule_id); }

 protected:
  PhaseRingBook() = default;

 private:
  virtual std::vector<PhaseRing::Id> DoGetPhaseRings() const = 0;

  virtual std::optional<PhaseRing> DoGetPhaseRing(const PhaseRing::Id& ring_id) const = 0;

  virtual std::optional<PhaseRing> DoFindPhaseRing(const RightOfWayRule::Id& rule_id) const = 0;

  virtual std::optional<PhaseRing> DoFindPhaseRing(const Rule::Id& rule_id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
