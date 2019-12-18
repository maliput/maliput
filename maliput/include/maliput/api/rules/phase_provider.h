#pragma once

#include <optional>

#include "drake/common/drake_copyable.h"

#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/state_provider_result.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the dynamic states (Phase::Id) of a
/// collection of PhaseRings.
class PhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseProvider);

  virtual ~PhaseProvider() = default;

  /// Result returned by GetPhase().
  using Result = StateProviderResult<Phase::Id>;

  /// Gets the phase within a specified PhaseRing. Returns std::nullopt if
  /// @p id is unrecognized.
  std::optional<Result> GetPhase(const PhaseRing::Id& id) const { return DoGetPhase(id); }

 protected:
  PhaseProvider() = default;

 private:
  virtual std::optional<Result> DoGetPhase(const PhaseRing::Id& id) const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
