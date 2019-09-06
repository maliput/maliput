#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "maliput/api/intersection.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/base/manual_phase_provider.h"

namespace maliput {

/// A concrete implementation of the api::Intersection abstract interface.
class Intersection : public api::Intersection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Intersection)

  /// Constructs an Intersection instance.
  ///
  /// @param id The intersection's unique ID.
  ///
  /// @param region The region of the road network that is part of the
  /// intersection.
  ///
  /// @param ring The PhaseRing that defines the phases within the intersection.
  ///
  /// @param phase_provider Enables the current phase within an
  /// api::rules::PhaseRing with ID @p ring_id to be specified and obtained. The
  /// pointer must remain valid throughout this class instance's lifetime.
  Intersection(const Id& id, const std::vector<api::LaneSRange>& region, const api::rules::PhaseRing& ring,
               ManualPhaseProvider* phase_provider);

  virtual ~Intersection() = default;

  drake::optional<api::rules::PhaseProvider::Result> Phase() const override;

  void SetPhase(const api::rules::Phase::Id& phase_id,
                const drake::optional<api::rules::Phase::Id>& next_phase = drake::nullopt,
                const drake::optional<double>& duration_until = drake::nullopt) override;

 private:
  ManualPhaseProvider* phase_provider_{};
};

}  // namespace maliput
