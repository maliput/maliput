// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <optional>
#include <vector>

#include "maliput/api/regions.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_provider.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"

namespace maliput {
namespace api {

/// An abstract convenience class that aggregates information pertaining to an
/// intersection. Its primary purpose is to serve as a single source of this
/// information and to remove the need for users to query numerous disparate
/// data structures and state providers.
class Intersection {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Intersection)

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
  virtual std::optional<rules::PhaseProvider::Result> Phase() const = 0;

  /// Sets the current Phase and optionally the next Phase.
  ///
  /// @throws std::exception if @p duration_until is defined when @p next_phase
  /// is undefined.
  virtual void SetPhase(const api::rules::Phase::Id& phase_id,
                        const std::optional<api::rules::Phase::Id>& next_phase = std::nullopt,
                        const std::optional<double>& duration_until = std::nullopt) = 0;

  /// Returns the region. See constructor parameter @p region for more details.
  const std::vector<LaneSRange>& region() const { return region_; }

  /// Returns the rules::PhaseRing::Id of the rules::PhaseRing that applies to
  /// this intersection. See constructor parameter @p ring for more details.
  const rules::PhaseRing::Id& ring_id() const { return ring_.id(); }

  /// Returns the current bulb states within the intersection.
  const std::optional<rules::BulbStates> bulb_states() const;

  /// Returns the current discrete value rule states within the intersection.
  const std::optional<rules::DiscreteValueRuleStates> DiscreteValueRuleStates() const;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// Returns the current RightOfWayRule states within the intersection.
  MALIPUT_DEPRECATED("RightOfWayRule class will be deprecated", "DiscreteValueRuleStates")
  const std::optional<rules::RuleStates> RuleStates() const;
#pragma GCC diagnostic pop

  /// Determines whether the rules::TrafficLight::Id is within this Intersection.
  ///
  /// @param id A rules::TrafficLight::Id.
  /// @returns True When `id` is within this Intersection.
  bool Includes(const rules::TrafficLight::Id& id) const;

  /// Determines whether the rules::DiscreteValueRule::Id is within this Intersection.
  ///
  /// @param id A rules::DiscreteValueRule::Id.
  /// @returns True When `id` is within this Intersection.
  bool Includes(const rules::DiscreteValueRule::Id& id) const;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// Determines whether the rules::RightOfWayRule::Id is within this Intersection.
  ///
  /// @param id A rules::RightOfWayRule::Id.
  /// @returns True When `id` is within this Intersection.
  MALIPUT_DEPRECATED("RightOfWayRule class will be deprecated")
  bool Includes(const rules::RightOfWayRule::Id& id) const;
#pragma GCC diagnostic pop

  /// Determines whether `inertial_position` is within this Intersection::Region().
  ///
  /// @param inertial_position A InertialPosition in the `Inertial`-frame.
  /// @param road_geometry The RoadGeometry where Intersection::Region() is contained. It must not be nullptr.
  /// @returns True When `inertial_position` is within Intersection::Region(). `inertial_position` is contained if
  /// the distance to the closest LanePosition in Intersection::Region() is less or equal than the linear tolerance of
  /// the `road_geometry`.
  ///
  /// @throws common::assertion_error When `road_geometry` is nullptr.
  /// @throws common::assertion_error When Lanes in Intersection::Region() are not found in `road_geometry`.
  bool Includes(const InertialPosition& inertial_position, const RoadGeometry* road_geometry) const;

  // TODO(liang.fok) Add method for obtaining the intersection's bounding box.
 private:
  const Id id_;
  const std::vector<LaneSRange> region_;
  const rules::PhaseRing ring_;
};

}  // namespace api
}  // namespace maliput
