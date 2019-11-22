#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/type_specific_identifier.h"

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
  Intersection(const Id& id, const std::vector<LaneSRange>& region, const rules::PhaseRing& ring);

  virtual ~Intersection() = default;

  /// Returns the persistent identifier.
  const Id& id() const { return id_; }

  /// Returns the current phase.
  virtual drake::optional<rules::PhaseProvider::Result> Phase() const = 0;

  /// Sets the current Phase and optionally the next Phase.
  ///
  /// @throws std::exception if @p duration_until is defined when @p next_phase
  /// is undefined.
  virtual void SetPhase(const api::rules::Phase::Id& phase_id,
                        const drake::optional<api::rules::Phase::Id>& next_phase = drake::nullopt,
                        const drake::optional<double>& duration_until = drake::nullopt) = 0;

  /// Returns the region. See constructor parameter @p region for more details.
  const std::vector<LaneSRange>& region() const { return region_; }

  /// Returns the rules::PhaseRing::Id of the rules::PhaseRing that applies to
  /// this intersection. See constructor parameter @p ring for more details.
  const rules::PhaseRing::Id& ring_id() const { return ring_.id(); }

  /// Returns the current bulb states within the intersection.
  const drake::optional<rules::BulbStates> bulb_states() const;

  /// Determines whether `geo_position` is included in this Intersection::Region().
  ///
  /// @param geo_position A GeoPosition of the World Frame.
  /// @param road_geometry The RoadGeometry where `region` is contained. It must not be nullptr.
  /// @returns True when `geo_position` is contained within the `region`. `geo_position` is contained if the distance to
  /// the closer LanePosition of `region` is minor or equal than the linear tolerance of the `road_geometry`.
  ///
  /// @throws common::assertion_error When `road_geometry` is nullptr.
  /// @throws common::assertion_error When Lanes of this Intersection::Region() are not found in `road_geometry`.
  bool Includes(const GeoPosition& geo_position, const RoadGeometry* road_geometry) const;

  // TODO(liang.fok) Add method for obtaining the intersection's bounding box.
 private:
  const Id id_;
  const std::vector<LaneSRange> region_;
  const rules::PhaseRing ring_;
};

}  // namespace api
}  // namespace maliput
