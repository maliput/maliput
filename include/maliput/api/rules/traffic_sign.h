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

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_hash.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/math/bounding_box.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace api {
namespace rules {

/// Defines the possible traffic sign types.
enum class TrafficSignType {
  kStop = 0,
  kYield,
  kSpeedLimit,
  kNoEntry,
  kOneWay,
  kPedestrianCrossing,
  kNoLeftTurn,
  kNoRightTurn,
  kNoUTurn,
  kSchoolZone,
  kConstruction,
  kRailroadCrossing,
  kUnknown,
};

/// Maps TrafficSignType enums to string representations.
std::unordered_map<TrafficSignType, const char*, maliput::common::DefaultHash> TrafficSignTypeMapper();

/// Models a physical traffic sign — a static, passive signaling device
/// placed along or above the road to convey regulatory, warning, or
/// informational messages to road users.
///
/// @note TrafficSign intentionally does not hold a reference to any Rule.
/// The linkage between a sign and its corresponding rule is established in the
/// opposite direction: a Rule's RelatedUniqueIds can reference a TrafficSign's
/// ID. This mirrors the existing TrafficLight <-> Rule coupling pattern.
class TrafficSign final {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficSign)

  /// Unique identifier for a TrafficSign.
  using Id = TypeSpecificIdentifier<class TrafficSign>;

  /// Constructs a TrafficSign instance.
  ///
  /// @param id The sign's unique identifier. It must be unique within the
  /// TrafficSignBook that contains this sign.
  ///
  /// @param type The type of sign (e.g., TrafficSignType::kStop,
  /// TrafficSignType::kYield, TrafficSignType::kSpeedLimit).
  ///
  /// @param position_road_network The linear offset of the sign's frame
  /// relative to the road network's Inertial frame. The origin should
  /// approximate the center of the sign face.
  ///
  /// @param orientation_road_network The rotational offset of the sign's
  /// frame relative to the road network's Inertial frame. The +X axis should
  /// point in the direction the sign faces (toward approaching vehicles), the
  /// +Z axis should point "up".
  ///
  /// @param message An optional text message displayed on the sign (e.g.,
  /// "60" on a speed limit sign, "SCHOOL ZONE", etc.). This provides
  /// supplementary information beyond what the type alone conveys.
  ///
  /// @param related_lanes The lanes that this sign is physically relevant to.
  /// For example, a stop sign at an intersection is relevant to the lanes
  /// that face it. This captures a spatial/geometric fact about sign
  /// placement, not a rule relationship. May be empty if the sign is not
  /// associated with specific lanes (e.g., a generic road-side sign).
  ///
  /// @param bounding_box The bounding box of the sign. The position and
  /// orientation are in the sign's local frame.
  TrafficSign(const Id& id, const TrafficSignType& type, const InertialPosition& position_road_network,
              const Rotation& orientation_road_network, const std::optional<std::string>& message,
              std::vector<LaneId> related_lanes, const maliput::math::BoundingBox& bounding_box);

  /// Returns this sign's unique identifier.
  const Id& id() const { return id_; }

  /// Returns the type of this sign.
  const TrafficSignType& type() const { return type_; }

  /// Returns the position of the sign's frame relative to the road network's
  /// Inertial frame.
  const InertialPosition& position_road_network() const { return position_road_network_; }

  /// Returns the orientation of the sign's frame relative to the road
  /// network's Inertial frame.
  const Rotation& orientation_road_network() const { return orientation_road_network_; }

  /// Returns the optional text message on the sign.
  const std::optional<std::string>& message() const { return message_; }

  /// Returns the lanes that this sign is physically relevant to.
  ///
  /// This captures which lanes the sign faces or is associated with from a
  /// spatial perspective. It is distinct from rule-lane associations: a rule's
  /// zone (LaneSRange) defines the precise region where the rule applies,
  /// while this field captures the physical placement relationship.
  const std::vector<LaneId>& related_lanes() const { return related_lanes_; }

  /// Returns the bounding box of this sign.
  const maliput::math::BoundingBox& bounding_box() const { return bounding_box_; }

 private:
  Id id_;
  TrafficSignType type_;
  InertialPosition position_road_network_;
  Rotation orientation_road_network_;
  std::optional<std::string> message_;
  std::vector<LaneId> related_lanes_;
  maliput::math::BoundingBox bounding_box_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
