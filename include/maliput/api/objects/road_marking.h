// BSD 3-Clause License
//
// Copyright (c) 2022-2026, Woven by Toyota. All rights reserved.
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

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/objects/road_object.h"
#include "maliput/api/traffic_control_device_type.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_hash.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/math/bounding_box.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace api {
namespace objects {

/// Road marking semantic type.
using RoadMarkingType = maliput::api::TrafficControlDeviceType;

/// Maps RoadMarkingType enums to string representations.
std::unordered_map<RoadMarkingType, const char*, maliput::common::DefaultHash> RoadMarkingTypeMapper();

/// Defines the unit for a road marking's numeric value.
enum class RoadMarkingValueUnit {
  kMetersPerSecond = 0,  ///< m/s
  kKilometersPerHour,    ///< km/h
  kMilesPerHour,         ///< mph
};

/// Maps RoadMarkingValueUnit enums to string representations.
std::unordered_map<RoadMarkingValueUnit, const char*, maliput::common::DefaultHash> RoadMarkingValueUnitMapper();

/// Holds a numeric value and its associated unit for a road marking.
///
/// For example, a speed limit marking might have value=60 with
/// unit=RoadMarkingValueUnit::kKilometersPerHour.
struct RoadMarkingValue {
  /// Equality operator.
  bool operator==(const RoadMarkingValue& other) const { return value == other.value && unit == other.unit; }
  /// Inequality operator.
  bool operator!=(const RoadMarkingValue& other) const { return !(*this == other); }

  /// The numeric value of the marking.
  double value{};
  /// The unit of the value.
  RoadMarkingValueUnit unit{RoadMarkingValueUnit::kMetersPerSecond};
};

/// Models a physical road marking — a painted or applied marking on the
/// road surface that conveys regulatory, warning, or guidance information
/// to road users.
///
/// Examples include stop markings, crosswalks, directional arrows, speed
/// limit markings, parking space delineations, and railroad crossings.
///
/// @note RoadMarking intentionally does not hold a reference to any Rule.
/// The linkage between a marking and its corresponding rule is established
/// in the opposite direction: a Rule's RelatedUniqueIds can reference a
/// RoadMarking's ID. This mirrors the existing TrafficLight <-> Rule and
/// TrafficSign <-> Rule coupling patterns.
class RoadMarking final {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadMarking)

  /// Unique identifier for a RoadMarking.
  using Id = TypeSpecificIdentifier<class RoadMarking>;

  /// Constructs a RoadMarking instance.
  ///
  /// @param id The marking's unique identifier. It must be unique within the
  /// RoadMarkingBook that contains this marking.
  ///
  /// @param type The type of marking (e.g., RoadMarkingType::kStop,
  /// RoadMarkingType::kCrosswalk, RoadMarkingType::kSpeedLimit).
  ///
  /// @param position The position of the marking relative to the road network.
  ///
  /// @param orientation The rotational offset of the marking's frame relative
  /// to the road network's Inertial frame.
  ///
  /// @param bounding_box The bounding box of the marking.
  ///
  /// @param related_lanes The lanes that this marking is physically relevant
  /// to. For example, a crosswalk marking is relevant to the lanes it spans.
  ///
  /// @param name An optional human-readable name for the marking.
  ///
  /// @param outlines Optional detailed shape outlines. When defined, they
  /// supersede the bounding box for precise geometry.
  ///
  /// @param value An optional numeric value associated with the marking (e.g.,
  /// 60 km/h for a speed limit marking).
  RoadMarking(const Id& id, RoadMarkingType type, const RoadObjectPosition& position, const Rotation& orientation,
              const maliput::math::BoundingBox& bounding_box, std::vector<LaneId> related_lanes,
              std::optional<std::string> name = std::nullopt, std::vector<std::unique_ptr<Outline>> outlines = {},
              const std::optional<RoadMarkingValue>& value = std::nullopt);

  /// Returns this marking's unique identifier.
  const Id& id() const { return id_; }

  /// Returns the optional human-readable name of this marking.
  std::optional<std::string> name() const { return name_; }

  /// Returns the type of this marking.
  RoadMarkingType type() const { return type_; }

  /// Returns the position of this marking relative to the road network.
  const RoadObjectPosition& position() const { return position_; }

  /// Returns the orientation of this marking in the inertial frame.
  const Rotation& orientation() const { return orientation_; }

  /// Returns the bounding box of this marking.
  const maliput::math::BoundingBox& bounding_box() const { return bounding_box_; }

  /// Returns the lanes that this marking is physically relevant to.
  const std::vector<LaneId>& related_lanes() const { return related_lanes_; }

  /// Returns the outlines defining the detailed shape of this marking.
  ///
  /// If outlines are defined, they supersede the bounding box for precise
  /// geometry representation.
  const std::vector<std::unique_ptr<Outline>>& outlines() const { return outlines_; }

  /// Returns the number of outlines.
  int num_outlines() const { return static_cast<int>(outlines_.size()); }

  /// Returns the outline at the specified index.
  /// @pre index >= 0 && index < num_outlines().
  const Outline* outline(int index) const;

  /// Returns the numeric value associated with this marking, if any.
  ///
  /// For example, a speed limit marking would return the speed value and its
  /// unit (e.g., {60.0, RoadMarkingValueUnit::kKilometersPerHour}).
  const std::optional<RoadMarkingValue>& GetValue() const { return value_; }

 private:
  Id id_;
  std::optional<std::string> name_;
  RoadMarkingType type_;
  RoadObjectPosition position_;
  Rotation orientation_;
  maliput::math::BoundingBox bounding_box_;
  std::vector<LaneId> related_lanes_;
  std::vector<std::unique_ptr<Outline>> outlines_;
  std::optional<RoadMarkingValue> value_;
};

}  // namespace objects
}  // namespace api
}  // namespace maliput
