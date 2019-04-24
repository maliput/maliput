#pragma once

#include <vector>

#include "maliput/api/intersection.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/regions.h"
#include "maliput/base/manual_phase_provider.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

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
  /// @param ring_id The ID of the ring that defines the phases within the
  /// intersection.
  ///
  /// @param phase_provider Enables the current phase within an
  /// api::rules::PhaseRing with ID @p ring_id to be specified and obtained. The
  /// pointer must remain valid throughout this class instance's lifetime.
  Intersection(const Id& id, const std::vector<api::rules::LaneSRange>& region,
               const api::rules::PhaseRing::Id& ring_id,
               ManualPhaseProvider* phase_provider);

  virtual ~Intersection() = default;

  const optional<api::rules::PhaseProvider::Result> Phase() const override;
  const drake::optional<api::rules::PhaseProvider::Result> Phase() const;

  /// Sets the current phase.
  void SetPhase(const api::rules::Phase::Id& phase_id) override;

 private:
  ManualPhaseProvider* phase_provider_{};
};

}  // namespace maliput
