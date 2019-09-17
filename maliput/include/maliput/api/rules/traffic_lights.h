#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/hash.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/type_specific_identifier.h"

namespace maliput {
namespace api {
namespace rules {

/// Defines the possible bulb colors.
enum class BulbColor {
  kRed = 0,
  kYellow,
  kGreen,
};

/// Maps BulbColor enums to string representations.
std::unordered_map<BulbColor, const char*, drake::DefaultHash> BulbColorMapper();

/// Defines the possible bulb types.
enum class BulbType {
  kRound = 0,
  kArrow,
};

/// Maps BulbType enums to string representations.
std::unordered_map<BulbType, const char*, drake::DefaultHash> BulbTypeMapper();

/// Defines the possible bulb states.
enum class BulbState { kOff = 0, kOn, kBlinking };

/// Maps BulbState enums to string representations.
std::unordered_map<BulbState, const char*, drake::DefaultHash> BulbStateMapper();

class BulbGroup;

/// Models a bulb within a bulb group.
class Bulb final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Bulb);

  /// Unique identifier for a Bulb.
  using Id = TypeSpecificIdentifier<class Bulb>;

  /// Defines the bounding box of the bulb. The bounding box's corners are
  /// interpreted as follows: Given two points, @p p_BMin and @p p_BMax), these
  /// are points in the bulb's frame B, such that the former is the minimum
  /// corner of the axis-aligned bounding box, and the latter is the maximum
  /// corner. The box should have the minimum volume needed to fully enclose the
  /// bulb.
  struct BoundingBox {
    /// The default constructor creates a bounding box based on the approximate
    /// dimensions of a bulb with a 12" lens, as shown by the following website:
    /// https://www.oksolar.com/images/signal_dimensions_large.jpg. These
    /// dimensions are 14" (0.356m) tall, 14" (0.356m) wide, and 7" (0.177m)
    /// deep. Note that the visor is not included as part of the depth since
    /// it is relatively thin and typically not obviously visible to approaching
    /// vehicles. Assuming the bulb frame's origin is in the middle of the light
    /// bulb's body, the bounding box has a `p_BMin` of (-3.5", -7", -7") and a
    /// `p_BMax` of (3.5", 7", 7"). In metric units, this is a `p_BMin` of
    /// (-0.0889m, -0.1778m, -0.1778m) and a `p_BMax` of (0.0889m, 0.1778m,
    /// 0.1778m).
    BoundingBox() : p_BMin(-0.0889, -0.1778, -0.1778), p_BMax(0.0889, 0.1778, 0.1778) {}
    Eigen::Vector3d p_BMin;
    Eigen::Vector3d p_BMax;
  };

  /// Constructs a Bulb instance.
  ///
  /// @param id The bulb's ID. It must be unique in the context of the BulbGroup
  ///        that contains it. @see UniqueBulbId to uniquely identify a Bulb in
  ///        the world.
  ///
  /// @param position_bulb_group The linear offset of this bulb's frame relative
  /// to the frame of the bulb group that contains it. The origin of this bulb's
  /// frame should approximate the bulb's CoM.
  ///
  /// @param orientation_bulb_group The rotational offset of this bulb's frame
  /// relative to the frame of the bulb group that contains it. The +Z axis
  /// should align with the bulb's "up" direction and the +X axis should point
  /// in the direction that the bulb is facing. Following a right-handed
  /// coordinate frame, the +Y axis should point left when facing the +X
  /// direction.
  ///
  /// @param color The color of this bulb.
  ///
  /// @param type The type of this bulb.
  ///
  /// @param arrow_orientation_rad The orientation of the arrow when @p type is
  /// BulbType::kArrow. This is the angle along the bulb's +X axis relative to
  /// the bulb's +Y axis. For example, an angle of zero means the bulb's arrow
  /// is pointing along the bulb's +Y axis, which means it's a right-turn arrow
  /// when viewed by an approaching vehicle. Similarly, an angle of PI/2 points
  /// in the bulb's +Z direction (i.e., forward from an approaching vehicle's
  /// perspective), and an angle of PI points to in the bulb's -Y direction
  /// (i.e., left from an approaching vehicle's perspective). This parameter
  /// must be defined when @p type is BulbType::kArrow, otherwise an exception
  /// will be thrown. An exception will also be thrown if this parameter is
  /// defined for non-arrow BulbType values.
  ///
  /// @param states The possible states of this bulb. If this is drake::nullopt or an
  /// empty vector, this bulb has states {BulbState::kOff, BulbState::kOn}.
  ///
  /// @param bounding_box The bounding box of the bulb. See BoundingBox for
  /// details about the default value.
  Bulb(const Id& id, const GeoPosition& position_bulb_group, const Rotation& orientation_bulb_group,
       const BulbColor& color, const BulbType& type,
       const drake::optional<double>& arrow_orientation_rad = drake::nullopt,
       const drake::optional<std::vector<BulbState>>& states = drake::nullopt,
       BoundingBox bounding_box = BoundingBox());

  /// Returns this Bulb instance's unique identifier.
  const Id& id() const { return id_; }

  /// Returns the linear offset of this bulb's frame relative to the frame of
  /// the bulb group that contains it.
  const GeoPosition& position_bulb_group() const { return position_bulb_group_; }

  /// Returns the rotational offset of this bulb's frame relative to the frame
  /// of the bulb group that contains it.
  const Rotation& orientation_bulb_group() const { return orientation_bulb_group_; }

  /// Returns the color of this bulb.
  const BulbColor& color() const { return color_; }

  /// Returns the type of this bulb.
  const BulbType& type() const { return type_; }

  /// Returns the arrow's orientation. Only applicable if type() returns
  /// BulbType::kArrow. See constructor's documentation for semantics.
  drake::optional<double> arrow_orientation_rad() const { return arrow_orientation_rad_; }

  /// Returns the possible states of this bulb.
  const std::vector<BulbState>& states() const { return states_; }

  /// Returns the default state of the bulb. The priority order is
  /// BulbState::kOff, BulbState::kBlinking, then BulbState::kOn. For example,
  /// if a bulb can be in states {BulbState::kOff, BulbState::kOn}, its default
  /// state will be BulbState::kOff. Likewise, if a bulb can be in states
  /// {BulbState::kOn, BulbState::kBlinking}, its default state will be
  /// BulbState::kBlinking. The only case where the default state is
  /// BulbState::kOn is when this is the bulb's only possible state.
  BulbState GetDefaultState() const;

  /// Returns true if @p bulb_state is one of this bulb's possible states.
  bool IsValidState(const BulbState& bulb_state) const;

  /// Returns the bounding box of the bulb.
  const BoundingBox& bounding_box() const { return bounding_box_; }

  /// Returns the World Frame Position of this bulb by transforming parent's
  /// BulbGroup frame and its parent TrafficLight frame with this bulb's
  /// nominal pose into World Frame.
  ///
  /// @throws common::assertion_error When this Bulb has not been assigned
  ///         to any BulbGroup.
  /// @throws common::assertion_error When this BulbGroup has not been
  ///         assigned to any TrafficLight.
  GeoPosition WorldFramePosition() const;

  /// Returns parent BulbGroup pointer.
  ///
  /// At parent BulbGroup construct time, the parent will set the pointer to
  /// this bulb via BulbGroupAttorney.
  /// In case this bulb is not contained by any BulbGroup, the return value
  /// will be nullptr.
  const BulbGroup* bulb_group() const { return bulb_group_; }

 private:
  // Sets parent BulbGroup pointer.
  //
  // @throws common::assertion_error When `bulb_group` is nullptr.
  void set_bulb_group(const BulbGroup* bulb_group) {
    MALIPUT_THROW_UNLESS(bulb_group != nullptr);
    bulb_group_ = bulb_group;
  }

  // Used to register a reference to the parent BulbGroup, at parent's
  // construct time via the attorney-client idiom.
  friend class BulbGroupAttorney;

  Id id_;
  GeoPosition position_bulb_group_;
  Rotation orientation_bulb_group_;
  BulbColor color_ = BulbColor::kRed;
  BulbType type_ = BulbType::kRound;
  drake::optional<double> arrow_orientation_rad_ = drake::nullopt;
  std::vector<BulbState> states_;
  BoundingBox bounding_box_;
  const BulbGroup* bulb_group_{};
};

class TrafficLight;

/// Models a group of bulbs within a traffic light. All of the bulbs within a
/// group should share the same approximate orientation. However, this is not
/// programmatically enforced.
class BulbGroup final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BulbGroup);

  /// Unique identifier for a BulbGroup.
  using Id = TypeSpecificIdentifier<BulbGroup>;

  /// Constructs a BulbGroup instance.
  ///
  /// @param id The bulb group's ID. It must be unique in the context of the
  ///        TrafficLight that contains it.
  ///
  /// @param position_traffic_light The linear offset of this bulb group's frame
  /// relative to the frame of the traffic light that contains it. The origin of
  /// this bulb group's frame should approximate the bulb group's CoM.
  ///
  /// @param orientation_traffic_light The rotational offset of this bulb
  /// group's frame relative to the frame of the traffic light that contains it.
  /// The +Z axis should align with the bulb group's "up" direction, and the +X
  /// axis should point in the direction that the bulb group is facing.
  /// Following a right-handed coordinate frame, the +Y axis should point left
  /// when facing the +X direction.
  ///
  /// @param bulbs The bulbs that are part of this BulbGroup. There must be at
  /// least one bulb within this group. There must not be Bulbs with the same
  /// Bulb::Id.
  ///
  /// @throws common::assertion_error When there are Bulbs with the same
  /// Bulb::Id in @p bulbs.
  BulbGroup(const Id& id, const GeoPosition& position_traffic_light, const Rotation& orientation_traffic_light,
            const std::vector<Bulb>& bulbs);

  /// Returns this BulbGroup instance's unique identifier.
  const Id& id() const { return id_; }

  /// Returns the linear offset of this bulb group's frame relative to the
  /// frame of the traffic light that contains it.
  const GeoPosition& position_traffic_light() const { return position_traffic_light_; }

  /// Returns the rotational offset of this bulb group's frame relative to the
  /// frame of the traffic light that contains it.
  const Rotation& orientation_traffic_light() const { return orientation_traffic_light_; }

  /// Returns the bulbs contained within this bulb group.
  const std::vector<Bulb>& bulbs() const { return bulbs_; }

  /// Gets the specified Bulb. Returns drake::nullopt if @p id is unrecognized.
  drake::optional<Bulb> GetBulb(const Bulb::Id& id) const;

  /// Returns the World Frame Position of this bulb by transforming parent's
  /// TrafficLight frame with this bulb group's nominal pose into World Frame.
  ///
  /// @throws common::assertion_error When this BulbGroup has not been assigned
  ///         to any TrafficLight.
  GeoPosition WorldFramePosition() const;

  /// Returns parent TrafficLight pointer.
  ///
  /// At parent TrafficLight construct time, the parent will set the pointer to
  /// this bulb via TrafficLightAttorney.
  /// In case this bulb group is not contained by any TrafficLight, the return
  /// value will be nullptr.
  const TrafficLight* traffic_light() const { return traffic_light_; }

 private:
  // Sets parent BulbGroup pointer.
  //
  // @throws common::assertion_error When `bulb_group` is nullptr.
  void set_traffic_light(const TrafficLight* traffic_light) {
    MALIPUT_THROW_UNLESS(traffic_light != nullptr);
    traffic_light_ = traffic_light;
  }

  // Used to register a reference to the parent BulbGroup, at parent's
  // construct time via the attorney-client idiom.
  friend class TrafficLightAttorney;

  Id id_;
  GeoPosition position_traffic_light_;
  Rotation orientation_traffic_light_;
  std::vector<Bulb> bulbs_;
  const TrafficLight* traffic_light_{};
};

/// Models a traffic light. A traffic light is a physical signaling device
/// typically located at road intersections. It contains one or more groups of
/// light bulbs with varying colors and shapes. The lighting patterns of the
/// bulbs signify right-of-way rule information to the agents navigating the
/// intersection (e.g., vehicles, bicyclists, pedestrians, etc.). Typically, an
/// intersection will be managed by multiple traffic lights.
///
/// Note that traffic lights are physical manifestations of underlying
/// right-of-way rules and thus naturally have lower signal-to-noise ratio
/// relative to the underlying rules. Thus, oracular agents should directly use
/// the underlying right-of-way rules instead of traffic lights when navigating
/// intersections. TrafficLight exists for testing autonomous vehicles that do
/// not have access to right-of-way rules.
class TrafficLight final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TrafficLight);

  /// Unique identifier for a traffic light.
  using Id = TypeSpecificIdentifier<TrafficLight>;

  /// Constructs a TrafficLight instance.
  ///
  /// @param id The traffic light's ID. It must be unique in the context of
  ///        the TrafficLightBook that contains it.
  ///
  /// @param position_road_network The linear offset of the traffic light's
  /// frame relative to the road network's frame. The traffic light frame's
  /// origin should approximate the traffic light's CoM.
  ///
  /// @param orientation_road_network The rotational offset of the traffic
  /// light's frame relative to the road network's frame. The traffic light's
  /// frame's +Z axis points in the traffic light's "up" direction. No
  /// constraints are placed on the orientations of the +X and +Y axes. However,
  /// it's recommended that they correspond, if possible, to the orientations of
  /// the bulb group frames within this traffic light. In particular, when the
  /// traffic light only has one bulb group, all three axes of both the traffic
  /// light and bulb group should ideally match, if possible.
  ///
  /// @param bulb_groups The bulb groups that are part of this traffic light.
  /// There must not be BulbGroups with the same BulbGroup::Ids.
  ///
  /// @throws common::assertion_error When there are BulbGroups with the same
  /// BulbGroup::Id in @p bulb_groups.
  TrafficLight(const Id& id, const GeoPosition& position_road_network, const Rotation& orientation_road_network,
               const std::vector<BulbGroup>& bulb_groups);

  /// Returns this traffic light's unique identifier.
  const Id& id() const { return id_; }

  /// Returns this traffic light's frame's position within the road network's
  /// frame.
  const GeoPosition& position_road_network() const { return position_road_network_; }

  const Rotation& orientation_road_network() const { return orientation_road_network_; }

  /// Returns the bulb groups contained within this traffic light.
  const std::vector<BulbGroup>& bulb_groups() const { return bulb_groups_; }

  /// Gets the specified BulbGroup. Returns drake::nullopt if @p id is unrecognized.
  drake::optional<BulbGroup> GetBulbGroup(const BulbGroup::Id& id) const;

 private:
  Id id_;
  GeoPosition position_road_network_;
  Rotation orientation_road_network_;
  std::vector<BulbGroup> bulb_groups_;
};

/// Allows BulbGroup registration in its Bulbs via the
/// client-attorney idiom.
class BulbGroupAttorney final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BulbGroupAttorney)
  BulbGroupAttorney() = delete;

 private:
  friend class BulbGroup;

  static void SetBulbGroupToBulb(const BulbGroup* bulb_group, Bulb* bulb);
};

/// Allows TrafficLight registration in its BulbGroups via the
/// client-attorney idiom.
class TrafficLightAttorney final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficLightAttorney)
  TrafficLightAttorney() = delete;

 private:
  friend class TrafficLight;

  static void SetTrafficLightToBulbGroup(const TrafficLight* traffic_light, BulbGroup* bulb_group);
};

/// Uniquely identifies a bulb in the world. This consists of the concatenation
/// of the bulb's ID, the ID of the bulb group that contains the bulb, and the
/// the ID of the traffic light that contains the bulb group.
struct UniqueBulbId {
  /// A default constructor. This was originally intended for use by
  /// boost::python bindings in downstream projects.
  UniqueBulbId()
      : traffic_light_id(TrafficLight::Id("default")),
        bulb_group_id(BulbGroup::Id("default")),
        bulb_id(Bulb::Id("default")){};

  /// Constructs a UniqueBulbId.
  UniqueBulbId(const TrafficLight::Id& traffic_light_id_in, const BulbGroup::Id& bulb_group_id_in,
               const Bulb::Id& bulb_id_in)
      : traffic_light_id(traffic_light_id_in), bulb_group_id(bulb_group_id_in), bulb_id(bulb_id_in) {}

  /// Returns the string representation of the %TypeSpecificIdentifier.
  const std::string to_string() const {
    return traffic_light_id.string() + "-" + bulb_group_id.string() + "-" + bulb_id.string();
  }

  /// Tests for equality with another UniqueBulbId.
  bool operator==(const UniqueBulbId& rhs) const {
    return traffic_light_id == rhs.traffic_light_id && bulb_group_id == rhs.bulb_group_id && bulb_id == rhs.bulb_id;
  }

  /// Tests for inequality with another UniqueBulbId, specifically
  /// returning the opposite of operator==().
  bool operator!=(const UniqueBulbId& rhs) const { return !(*this == rhs); }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const UniqueBulbId& id) noexcept {
    using drake::hash_append;
    hash_append(hasher, id.traffic_light_id);
    hash_append(hasher, id.bulb_group_id);
    hash_append(hasher, id.bulb_id);
  }

  TrafficLight::Id traffic_light_id;
  BulbGroup::Id bulb_group_id;
  Bulb::Id bulb_id;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput

namespace std {

/// Specialization of std::hash for maliput::api::rules::UniqueBulbId.
template <>
struct hash<maliput::api::rules::UniqueBulbId> : public drake::DefaultHash {};

/// Specialization of std::less for maliput::api::rules::UniqueBulbId
/// providing a strict ordering over maliput::api::rules::UniqueBulbId
/// suitable for use with ordered containers.
template <>
struct less<maliput::api::rules::UniqueBulbId> {
  bool operator()(const maliput::api::rules::UniqueBulbId& lhs, const maliput::api::rules::UniqueBulbId& rhs) const {
    if (lhs.traffic_light_id.string() < rhs.traffic_light_id.string()) {
      return true;
    }
    if (lhs.traffic_light_id.string() > rhs.traffic_light_id.string()) {
      return false;
    }
    if (lhs.bulb_group_id.string() < rhs.bulb_group_id.string()) return true;
    if (lhs.bulb_group_id.string() > rhs.bulb_group_id.string()) return false;
    return lhs.bulb_id.string() < rhs.bulb_id.string();
  }
};

}  // namespace std
