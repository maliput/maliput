#pragma once

#include <optional>

#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/right_of_way_rule_state_provider.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {

/// Provides the state of api::rules::RightOfWayRule instances based on
/// the current api::rules::Phase. The states of the rules that govern an
/// intersection are organized into phases. Each phase typically assigns
/// different states to each rule to ensure intersection safety and fairness.
/// For example, given an intersection between streets A and B governed by
/// Rule_A and Rule_B, respectively, two phases are necessary:
///
/// Phase  | Rule_A State | Rule_B State
/// ------ | ------------ | ------------
///   1    | GO           | STOP
///   2    | STOP         | GO
///
/// The rules above will ensure vehicles traveling on Street A do not collide
/// with vehicles traveling on Street B and vice versa.
class PhaseBasedRightOfWayRuleStateProvider final : public api::rules::RightOfWayRuleStateProvider {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseBasedRightOfWayRuleStateProvider)

  /// Constructs a PhaseBasedRuleStateProvider.
  ///
  /// All pointer parameters are aliased; they must not be nullptr and their
  /// lifespans must exceed that of this instance.
  PhaseBasedRightOfWayRuleStateProvider(const api::rules::PhaseRingBook* phase_ring_book,
                                        const api::rules::PhaseProvider* phase_provider);

  ~PhaseBasedRightOfWayRuleStateProvider() final = default;

  const api::rules::PhaseRingBook& phase_ring_book() const { return *phase_ring_book_; }

  const api::rules::PhaseProvider& phase_provider() const { return *phase_provider_; }

 private:
  std::optional<api::rules::RightOfWayRuleStateProvider::RightOfWayResult> DoGetState(
      const api::rules::RightOfWayRule::Id& id) const final;

  const api::rules::PhaseRingBook* phase_ring_book_{};
  const api::rules::PhaseProvider* phase_provider_{};
};

}  // namespace maliput
