#pragma once

#include <vector>

#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/regions.h"
#include "maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace maliput {
namespace api {

/// An abstract convenience class that aggregates information pertaining to an
/// intersection. Its primary purpose is to serve as a single source of this
/// information and to remove the need for users to query numerous disparate
/// data structures and state providers.
class Intersection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Intersection)

  /// Unique identifier for an Intersection.
  using Id = TypeSpecificIdentifier<class Intersection>;

  /// Constructs an Intersection instance.
  ///
  /// @param id The intersection's unique ID.
  ///
  /// @param region The region of the road network that should be considered
  /// part of the intersection.
  ///
  /// @param ring The PhaseRing that defines the phases within the intersection.
  Intersection(const Id& id, const std::vector<rules::LaneSRange>& region,
               const rules::PhaseRing& ring);

  virtual ~Intersection() = default;

  /// Returns the persistent identifier.
  const Id& id() const { return id_; }

  /// Returns the current phase.
  virtual drake::optional<rules::PhaseProvider::Result> Phase() const = 0;

  virtual void SetPhase(const api::rules::Phase::Id& phase_id) = 0;

  /// Returns the region. See constructor parameter @p region for more details.
  const std::vector<rules::LaneSRange>& region() const { return region_; }

  /// Returns the rules::PhaseRing::Id of the rules::PhaseRing that applies to
  /// this intersection. See constructor parameter @p ring for more details.
  const rules::PhaseRing::Id& ring_id() const { return ring_.id(); }

  /// Returns the current bulb states within the intersection.
  const drake::optional<rules::BulbStates> bulb_states() const;

  // TODO(liang.fok) Add method for obtaining the intersection's bounding box.
 private:
  const Id id_;
  const std::vector<rules::LaneSRange> region_;
  const rules::PhaseRing ring_;
};

}  // namespace api
}  // namespace maliput
