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

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_hash.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace api {
namespace objects {

/// Defines the type of road object.
///
/// These types are backend-agnostic categories that can represent objects
/// from various road description formats (e.g., OpenDRIVE).
enum class RoadObjectType {
  kUnknown = 0,    ///< Unknown or unclassified object.
  kBarrier,        ///< Continuous roadside barrier (guard rail, wall, fence, etc.).
  kBuilding,       ///< Building or permanent structure.
  kCrosswalk,      ///< Pedestrian or bicycle crossing.
  kGantry,         ///< Overhead structure for mounting signals.
  kObstacle,       ///< Static obstacle that cannot be passed.
  kParkingSpace,   ///< Designated parking area.
  kPole,           ///< Vertical pole structure (street lamp, traffic sign pole, etc.).
  kRoadMark,       ///< Painted road marking (arrows, symbols, text).
  kRoadSurface,    ///< Road surface element (manhole, speed bump, etc.).
  kTrafficIsland,  ///< Traffic island or median.
  kTree,           ///< Individual tree.
  kVegetation,     ///< Vegetation area (bush, forest, hedge).
};

/// Maps RoadObjectType enums to string representations.
std::unordered_map<RoadObjectType, const char*, maliput::common::DefaultHash> RoadObjectTypeMapper();

/// Defines the orientation validity of a road object relative to the road direction.
enum class RoadObjectOrientation {
  kBoth,      ///< Valid in both directions (default).
  kPositive,  ///< Valid only in positive s-direction.
  kNegative,  ///< Valid only in negative s-direction.
};

/// Maps RoadObjectOrientation enums to string representations.
std::unordered_map<RoadObjectOrientation, const char*, maliput::common::DefaultHash> RoadObjectOrientationMapper();

/// Represents an axis-aligned bounding box for a road object.
///
/// The bounding box is defined by its dimensions (length, width, height)
/// centered at the object's position. For circular objects, use radius instead.
class BoundingBox {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BoundingBox)

  /// Constructs an angular (box-shaped) bounding box.
  ///
  /// @param length Length along the object's local u-axis (forward direction).
  /// @param width Width along the object's local v-axis (lateral direction).
  /// @param height Height along the object's local z-axis.
  static BoundingBox Angular(double length, double width, double height);

  /// Constructs a circular bounding box.
  ///
  /// @param radius Radius in the u-v plane.
  /// @param height Height along the object's local z-axis.
  static BoundingBox Circular(double radius, double height);

  /// Default constructor creates an empty bounding box.
  BoundingBox() = default;

  /// Returns true if this is an angular (box-shaped) bounding box.
  bool is_angular() const { return !is_circular_; }

  /// Returns true if this is a circular bounding box.
  bool is_circular() const { return is_circular_; }

  /// Returns the length for angular bounding boxes.
  /// @pre is_angular() must be true.
  double length() const;

  /// Returns the width for angular bounding boxes.
  /// @pre is_angular() must be true.
  double width() const;

  /// Returns the radius for circular bounding boxes.
  /// @pre is_circular() must be true.
  double radius() const;

  /// Returns the height of the bounding box.
  double height() const { return height_; }

 private:
  BoundingBox(double length_or_radius, double width, double height, bool is_circular);

  double length_or_radius_{0.};
  double width_{0.};
  double height_{0.};
  bool is_circular_{false};
};

/// Represents a corner point in an object's outline.
///
/// Corners can be defined either in road coordinates (s, t, dz) relative to
/// the road reference line, or in local coordinates (u, v, z) relative to
/// the object's origin.
class OutlineCorner {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(OutlineCorner)

  /// Constructs an OutlineCorner at the given local position.
  ///
  /// @param position Position in local (u, v, z) coordinates.
  /// @param height Optional height at this corner (for 3D outlines).
  OutlineCorner(const math::Vector3& position, std::optional<double> height = std::nullopt)
      : position_(position), height_(height) {}

  /// Returns the position of this corner.
  const math::Vector3& position() const { return position_; }

  /// Returns the height at this corner, if defined.
  std::optional<double> height() const { return height_; }

 private:
  math::Vector3 position_;
  std::optional<double> height_;
};

/// Represents the outline of a road object.
///
/// An outline is a sequence of corner points that define the 2D or 3D shape
/// of an object. Outlines can be closed (forming a polygon) or open (forming
/// a polyline).
class Outline {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Outline)

  /// Unique identifier for an Outline within a RoadObject.
  using Id = TypeSpecificIdentifier<class Outline>;

  /// Constructs an Outline.
  ///
  /// @param id Unique identifier for this outline within the object.
  /// @param corners The sequence of corner points defining the outline.
  /// @param closed Whether the outline forms a closed polygon.
  Outline(const Id& id, std::vector<OutlineCorner> corners, bool closed = true);

  /// Returns the outline's identifier.
  const Id& id() const { return id_; }

  /// Returns the corners defining this outline.
  const std::vector<OutlineCorner>& corners() const { return corners_; }

  /// Returns true if this outline is closed (forms a polygon).
  bool is_closed() const { return closed_; }

  /// Returns the number of corners in this outline.
  int num_corners() const { return static_cast<int>(corners_.size()); }

 private:
  Id id_;
  std::vector<OutlineCorner> corners_;
  bool closed_;
};

/// Represents the position of a road object in the road network.
///
/// Objects are positioned using either:
/// - Inertial frame coordinates (x, y, z) for absolute positioning
/// - Optionally, a lane-relative position for objects associated with a specific lane
///
/// The inertial position is always available and serves as the primary position
/// reference. Lane-relative positioning is optional and useful for objects that
/// are semantically associated with a particular lane (e.g., a parking space).
class RoadObjectPosition {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RoadObjectPosition)

  /// Constructs a RoadObjectPosition with only inertial coordinates.
  ///
  /// Use this constructor for objects that are not associated with a specific lane.
  ///
  /// @param inertial_position The position in the inertial frame.
  explicit RoadObjectPosition(const InertialPosition& inertial_position);

  /// Constructs a RoadObjectPosition with both inertial and lane-relative coordinates.
  ///
  /// Use this constructor for objects associated with a specific lane.
  ///
  /// @param inertial_position The position in the inertial frame.
  /// @param lane_id The identifier of the lane this object is associated with.
  /// @param lane_position The position in the lane's (s, r, h) frame.
  RoadObjectPosition(const InertialPosition& inertial_position, const LaneId& lane_id,
                     const LanePosition& lane_position);

  /// Returns the position in the inertial frame.
  const InertialPosition& inertial_position() const { return inertial_position_; }

  /// Returns true if this position has an associated lane.
  bool has_lane_position() const { return lane_id_.has_value(); }

  /// Returns the lane identifier, if this object is associated with a lane.
  std::optional<LaneId> lane_id() const { return lane_id_; }

  /// Returns the lane-relative position, if this object is associated with a lane.
  std::optional<LanePosition> lane_position() const { return lane_position_; }

 private:
  InertialPosition inertial_position_;
  std::optional<LaneId> lane_id_;
  std::optional<LanePosition> lane_position_;
};

/// Models a static road object or piece of road furniture.
///
/// Road objects are items that influence a road by expanding, delimiting, or
/// supplementing its course. Examples include barriers, buildings, parking
/// spaces, traffic islands, poles, crosswalks, and vegetation.
///
/// Objects do not change their position in the road network. They may be
/// "dynamic" in the sense of having movable parts (e.g., fans, windmills),
/// but their base position remains fixed.
class RoadObject {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadObject)

  /// Unique identifier for a RoadObject.
  using Id = TypeSpecificIdentifier<class RoadObject>;

  virtual ~RoadObject() = default;

  /// Returns the unique identifier for this object.
  const Id& id() const { return id_; }

  /// Returns the human-readable name of this object, if provided.
  std::optional<std::string> name() const { return name_; }

  /// Returns the type of this object.
  RoadObjectType type() const { return type_; }

  /// Returns the subtype string for this object, if provided.
  ///
  /// Subtypes provide additional classification within a type.
  /// For example, a barrier might have subtype "guardRail" or "wall".
  std::optional<std::string> subtype() const { return subtype_; }

  /// Returns the position of this object relative to its associated road.
  const RoadObjectPosition& position() const { return position_; }

  /// Returns the orientation of this object in the inertial frame.
  const Rotation& orientation() const { return orientation_; }

  /// Returns the heading angle (yaw) relative to the road direction, in radians.
  double heading() const { return heading_; }

  /// Returns the pitch angle relative to the road surface, in radians.
  double pitch() const { return pitch_; }

  /// Returns the roll angle relative to the road surface, in radians.
  double roll() const { return roll_; }

  /// Returns the bounding box of this object.
  const BoundingBox& bounding_box() const { return bounding_box_; }

  /// Returns the orientation validity of this object.
  RoadObjectOrientation road_orientation() const { return road_orientation_; }

  /// Returns whether this object has movable parts (dynamic).
  ///
  /// Dynamic objects are fixed in position but may have animated elements.
  bool is_dynamic() const { return is_dynamic_; }

  /// Returns the validity length along the s-axis.
  ///
  /// A value of 0.0 indicates a point object. For extended objects like
  /// barriers, this represents the length over which the object is valid.
  double valid_length() const { return valid_length_; }

  /// Returns the outlines defining the detailed shape of this object.
  ///
  /// If outlines are defined, they supersede the bounding box for precise
  /// geometry representation.
  const std::vector<std::unique_ptr<Outline>>& outlines() const { return outlines_; }

  /// Returns the number of outlines.
  int num_outlines() const { return static_cast<int>(outlines_.size()); }

  /// Returns the outline at the specified index.
  /// @pre index >= 0 && index < num_outlines().
  const Outline* outline(int index) const;

  /// Returns backend-specific properties as key-value pairs.
  ///
  /// This provides access to additional properties that may be specific to
  /// a particular backend implementation (e.g., OpenDRIVE-specific attributes).
  const std::unordered_map<std::string, std::string>& properties() const { return properties_; }

 protected:
  /// Constructs a RoadObject.
  ///
  /// @param id Unique identifier for this object.
  /// @param type The type category of this object.
  /// @param position Position relative to the associated road.
  /// @param orientation Orientation in the inertial frame.
  /// @param heading Heading angle relative to road direction (radians).
  /// @param pitch Pitch angle (radians).
  /// @param roll Roll angle (radians).
  /// @param bounding_box The bounding box dimensions.
  /// @param road_orientation Orientation validity relative to road direction.
  /// @param is_dynamic Whether the object has movable parts.
  /// @param valid_length Validity length along s-axis.
  /// @param name Optional human-readable name.
  /// @param subtype Optional subtype classification.
  /// @param outlines Optional detailed shape outlines.
  /// @param properties Optional backend-specific properties.
  RoadObject(const Id& id, RoadObjectType type, const RoadObjectPosition& position, const Rotation& orientation,
             double heading, double pitch, double roll, const BoundingBox& bounding_box,
             RoadObjectOrientation road_orientation, bool is_dynamic, double valid_length,
             std::optional<std::string> name, std::optional<std::string> subtype,
             std::vector<std::unique_ptr<Outline>> outlines, std::unordered_map<std::string, std::string> properties);

 private:
  Id id_;
  std::optional<std::string> name_;
  RoadObjectType type_;
  std::optional<std::string> subtype_;
  RoadObjectPosition position_;
  Rotation orientation_;
  double heading_;
  double pitch_;
  double roll_;
  BoundingBox bounding_box_;
  RoadObjectOrientation road_orientation_;
  bool is_dynamic_;
  double valid_length_;
  std::vector<std::unique_ptr<Outline>> outlines_;
  std::unordered_map<std::string, std::string> properties_;
};

}  // namespace objects
}  // namespace api
}  // namespace maliput

namespace std {

/// Specialization of std::hash for maliput::api::objects::RoadObjectType.
template <>
struct hash<maliput::api::objects::RoadObjectType> : public maliput::common::DefaultHash {};

/// Specialization of std::hash for maliput::api::objects::RoadObjectOrientation.
template <>
struct hash<maliput::api::objects::RoadObjectOrientation> : public maliput::common::DefaultHash {};

}  // namespace std
