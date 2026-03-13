// BSD 3-Clause License
//
// Copyright (c) 2026, Woven Planet. All rights reserved.
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
#include <string>
#include <unordered_map>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace furniture {

/// Directional validity of a road furniture item relative to the road
/// reference line's s-direction.
enum class RoadFurnitureOrientation {
  kBoth,      ///< Applies to both directions of travel.
  kPositive,  ///< Applies only in the positive s-direction.
  kNegative,  ///< Applies only in the negative s-direction.
};

/// Unified type taxonomy for all road furniture items (signs and objects).
///
/// Sign-type values carry the `Sign` suffix to distinguish them from
/// object-type values. Backend-specific details (country code, raw XODR
/// type string) remain in RoadFurniture::properties().
enum class RoadFurnitureType {
  kUnknown = 0,      ///< Unknown or unclassified item.
  // --- Sign types ---
  kSpeedLimitSign,   ///< Speed limit (maximum or advisory).
  kStopSign,         ///< Mandatory stop.
  kYieldSign,        ///< Yield / give-way.
  kNoEntrySign,      ///< No entry in this direction.
  kWarningSign,      ///< Generic warning.
  kInformationSign,  ///< Informational or directional sign.
  // --- Object types ---
  kBarrier,
  kBuilding,
  kCrosswalk,
  kGantry,
  kObstacle,
  kParkingSpace,
  kPole,
  kRoadMark,
  kRoadSurface,
  kTrafficIsland,
  kTree,
  kVegetation,
};

// ---------------------------------------------------------------------------
// RoadFurniture — base class
// ---------------------------------------------------------------------------

/// Abstract base class for all static road furniture.
///
/// RoadFurniture is the backend-agnostic representation of any item of road
/// furniture: a traffic sign, a physical road object, or any other static
/// element associated with the road network.
///
/// The furniture_type() accessor exposes the item's high-level category
/// through the unified RoadFurnitureType enum.
class RoadFurniture {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadFurniture)
  using Id = TypeSpecificIdentifier<class RoadFurniture>;

  virtual ~RoadFurniture() = default;

  /// Returns the unique identifier.
  const Id& id() const { return DoId(); }

  /// Returns the human-readable name, if provided.
  std::optional<std::string> name() const { return DoName(); }

  /// Returns the high-level furniture category.
  ///
  /// Sign-type values carry the `Sign` suffix (e.g., kSpeedLimitSign).
  RoadFurnitureType furniture_type() const { return DoFurnitureType(); }

  /// Returns the position in the inertial frame.
  const InertialPosition& position() const { return DoPosition(); }

  /// Returns the 3D orientation.
  const Rotation& orientation() const { return DoOrientation(); }

  /// Returns the directional validity relative to the road reference line.
  RoadFurnitureOrientation road_orientation() const { return DoRoadOrientation(); }

  /// Returns the lane IDs to which this furniture item applies.
  ///
  /// Applicability is determined by the backend from lane-validity records
  /// (e.g., XODR <validity>) and the orientation field.
  std::vector<LaneId> affected_lane_ids() const { return DoAffectedLaneIds(); }

  /// Returns the IDs of rules linked to this furniture item.
  ///
  /// Callers may look up the actual rule objects in RoadRulebook.
  /// Returns an empty vector if no rules are associated.
  std::vector<rules::Rule::Id> GetRules() const { return DoGetRules(); }

  /// Returns backend-specific key-value attributes.
  ///
  /// Typical keys for XODR backends: "country", "xodr_type", "xodr_subtype",
  /// "country_revision". Keys are backend-defined strings.
  const std::unordered_map<std::string, std::string>& properties() const { return DoProperties(); }

 protected:
  RoadFurniture() = default;

 private:
  virtual const Id& DoId() const = 0;
  virtual std::optional<std::string> DoName() const = 0;
  virtual RoadFurnitureType DoFurnitureType() const = 0;
  virtual const InertialPosition& DoPosition() const = 0;
  virtual const Rotation& DoOrientation() const = 0;
  virtual RoadFurnitureOrientation DoRoadOrientation() const = 0;
  virtual std::vector<LaneId> DoAffectedLaneIds() const = 0;
  virtual std::vector<rules::Rule::Id> DoGetRules() const = 0;
  virtual const std::unordered_map<std::string, std::string>& DoProperties() const = 0;

  // TODO(#road-furniture): Add DoGetBoundingBox() and DoGetOutlines() NVI methods once
  // the geometric helper types (BoundingBox, Outline, OutlineCorner) are
  // migrated from maliput::api::objects into maliput::api::furniture.
};

}  // namespace furniture
}  // namespace api
}  // namespace maliput
