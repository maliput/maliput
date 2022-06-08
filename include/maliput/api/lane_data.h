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

#include <functional>
#include <ostream>
#include <string>

#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/math/matrix.h"
#include "maliput/math/quaternion.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace api {

class Lane;

/// A specific endpoint of a specific Lane.
struct LaneEnd {
  /// Labels for the endpoints of a Lane.
  /// kStart is the "s == 0" end, and kFinish is the other end.
  enum Which {
    kStart,
    kFinish,
  };

  /// An arbitrary strict complete ordering, useful for, e.g., std::map.
  struct StrictOrder {
    bool operator()(const LaneEnd& lhs, const LaneEnd& rhs) const {
      auto as_tuple = [](const LaneEnd& le) { return std::tie(le.lane, le.end); };
      return as_tuple(lhs) < as_tuple(rhs);
    }
  };

  /// Default constructor.
  LaneEnd() = default;

  /// Construct a LaneEnd specifying the @p end of @p lane.
  LaneEnd(const Lane* _lane, Which _end) : lane(_lane), end(_end) {}

  const Lane* lane{};
  Which end{};
};

/// Streams a string representation of @p which_end into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const LaneEnd::Which& which_end);

/// A position in 3-dimensional geographical Cartesian space, i.e., in the
/// `Inertial`-frame, consisting of three components x, y, and z.
class InertialPosition {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(InertialPosition)

  /// Default constructor, initializing all components to zero.
  InertialPosition() : xyz_(0., 0., 0.) {}

  /// Fully parameterized constructor.
  InertialPosition(double x, double y, double z) : xyz_(x, y, z) {}

  /// Constructs a InertialPosition from a 3-vector @p xyz of the form `[x, y, z]`.
  static InertialPosition FromXyz(const math::Vector3& xyz) { return InertialPosition(xyz); }

  /// Returns all components as 3-vector `[x, y, z]`.
  const math::Vector3& xyz() const { return xyz_; }

  /// Sets all components from 3-vector `[x, y, z]`.
  void set_xyz(const math::Vector3& xyz) { xyz_ = xyz; }

  /// @name Getters and Setters
  //@{
  /// Gets `x` value.
  double x() const { return xyz_.x(); }
  /// Sets `x` value.
  void set_x(double x) { xyz_.x() = x; }
  /// Gets `y` value.
  double y() const { return xyz_.y(); }
  /// Sets `y` value.
  void set_y(double y) { xyz_.y() = y; }
  /// Gets `z` value.
  double z() const { return xyz_.z(); }
  /// Sets `z` value.
  void set_z(double z) { xyz_.z() = z; }
  //@}

  /// Returns L^2 norm of 3-vector.
  double length() const { return xyz_.norm(); }

  /// Return the Cartesian distance to inertial_position.
  double Distance(const InertialPosition& inertial_position) const;

  /// Equality operator.
  bool operator==(const InertialPosition& rhs) const { return (this->xyz() == rhs.xyz()); }

  /// Inequality operator.
  bool operator!=(const InertialPosition& rhs) const { return !(this->xyz() == rhs.xyz()); }

  /// Plus operator.
  InertialPosition operator+(const InertialPosition& rhs) const {
    return InertialPosition::FromXyz(this->xyz() + rhs.xyz());
  }

  /// Minus operator.
  InertialPosition operator-(const InertialPosition& rhs) const {
    return InertialPosition::FromXyz(this->xyz() - rhs.xyz());
  }

  /// Multiplication by scalar operator.
  friend InertialPosition operator*(double lhs, const InertialPosition& rhs) {
    return InertialPosition::FromXyz(lhs * rhs.xyz());
  }

  /// Multiplication by scalar operator.
  InertialPosition operator*(double rhs) const { return InertialPosition::FromXyz(this->xyz() * rhs); }

 private:
  explicit InertialPosition(const math::Vector3& xyz) : xyz_(xyz) {}

  math::Vector3 xyz_;
};

/// Streams a string representation of @p inertial_position into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const InertialPosition& inertial_position);

/// A 3-dimensional rotation.
class Rotation {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Rotation)

  /// Default constructor, creating an identity Rotation.
  Rotation() : quaternion_(math::Quaternion::Identity()) {}

  /// Constructs a Rotation from a math::Quaternion @p quaternion (which will be
  /// normalized).
  static Rotation FromQuat(const math::Quaternion& quaternion) { return Rotation(quaternion.normalized()); }

  /// Constructs a Rotation from @p rpy, a vector of `[roll, pitch, yaw]`,
  /// expressing a roll around X, followed by pitch around Y,
  /// followed by yaw around Z (with all angles in radians).
  static Rotation FromRpy(const math::Vector3& rpy) {
    return FromQuat(math::RollPitchYaw(rpy.x(), rpy.y(), rpy.z()).ToQuaternion());
  }

  /// Constructs a Rotation expressing a @p roll around X, followed by
  /// @p pitch around Y, followed by @p yaw around Z (with all angles
  /// in radians).
  static Rotation FromRpy(double roll, double pitch, double yaw) { return FromRpy(math::Vector3(roll, pitch, yaw)); }

  /// Provides a quaternion representation of the rotation.
  const math::Quaternion& quat() const { return quaternion_; }

  /// Sets value from a math::Quaternion @p quaternion (which will be normalized).
  void set_quat(const math::Quaternion& quaternion) { quaternion_ = quaternion.normalized(); }

  /// Provides a 3x3 rotation matrix representation of "this" rotation.
  math::Matrix3 matrix() const;

  /// Provides a representation of rotation as a vector of angles
  /// `[roll, pitch, yaw]` (in radians).
  math::RollPitchYaw rpy() const { return math::RollPitchYaw(quaternion_); }

  // TODO(maddog@tri.global)  Deprecate and/or remove roll()/pitch()/yaw(),
  //                          since they hide the call to rpy(), and since
  //                          most call-sites should probably be using something
  //                          else (e.g., quaternion) anyway.
  /// Returns the roll component of the Rotation (in radians).
  double roll() const { return rpy().roll_angle(); }

  /// Returns the pitch component of the Rotation (in radians).
  double pitch() const { return rpy().pitch_angle(); }

  /// Returns the yaw component of the Rotation (in radians).
  double yaw() const { return rpy().yaw_angle(); }

  /// Returns the rotated `inertial_position` InertialPosition in the `Inertial`-frame.
  InertialPosition Apply(const InertialPosition& inertial_position) const;

  /// Returns the rotation corresponding to "going the other way instead", i.e.,
  /// the orientation of (-s,-r,h). This is equivalent to a pre-rotation by PI
  /// in the s/r plane.
  Rotation Reverse() const;

  /// Let $R_1$ be `this` rotation description, and let `rot` be $R_2$, another
  /// rotation description in the `Inertial`-frame. Let $F_W_1$ and $F_W_2$ be the
  /// versors describing the basis of the `Inertial`-frame. Then, let $F_R_1$ and
  /// $F_R_2$ be the rotated $F_W_1$ and $F_W_2$ by $R_1$ and $R_2$ respectively.
  /// This method returns the root square sum of each angle between
  /// versors in $F_R_1$ and $F_R_2$.
  double Distance(const Rotation& rot) const;

 private:
  explicit Rotation(const math::Quaternion& quat) : quaternion_(quat) {}

  math::Quaternion quaternion_;
};

/// Streams a string representation of @p rotation into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const Rotation& rotation);

/// A 3-dimensional position in a `Lane`-frame, consisting of three components:
///
/// * s is longitudinal position, as arc-length along a Lane's reference line.
/// * r is lateral position, perpendicular to the reference line at s. +r is to
///   to the left when traveling in the direction of +s.
/// * h is height above the road surface.
class LanePosition {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LanePosition)

  /// Default constructor, initializing all components to zero.
  LanePosition() : srh_(0., 0., 0.) {}

  /// Fully parameterized constructor.
  LanePosition(double s, double r, double h) : srh_(s, r, h) {}

  /// Constructs a LanePosition from a 3-vector @p srh of the form `[s, r, h]`.
  static LanePosition FromSrh(const math::Vector3& srh) { return LanePosition(srh); }

  /// Returns all components as 3-vector `[s, r, h]`.
  const math::Vector3& srh() const { return srh_; }

  /// Sets all components from 3-vector `[s, r, h]`.
  void set_srh(const math::Vector3& srh) { srh_ = srh; }

  /// @name Getters and Setters
  //@{
  /// Gets `s` value.
  double s() const { return srh_.x(); }
  /// Sets `s` value.
  void set_s(double s) { srh_.x() = s; }
  /// Gets `r` value.
  double r() const { return srh_.y(); }
  /// Sets `r` value.
  void set_r(double r) { srh_.y() = r; }
  /// Gets `h` value.
  double h() const { return srh_.z(); }
  /// Sets `h` value.
  void set_h(double h) { srh_.z() = h; }
  //@}

 private:
  explicit LanePosition(const math::Vector3& srh) : srh_(srh) {}

  math::Vector3 srh_;
};

/// Included in the return result of Lane::ToLanePosition().
struct LanePositionResult {
  /// The candidate LanePosition within the Lane' segment-bounds which
  /// is closest to a `inertial_position` supplied to Lane::ToLanePosition()
  /// (measured by the Cartesian metric in the `Inertial`-frame).
  LanePosition lane_position;
  /// The position that exactly corresponds to `lane_position`.
  InertialPosition nearest_position;
  /// The Cartesian distance between `nearest_position` and the
  /// `inertial_position` supplied to Lane::ToLanePosition().
  double distance{};
};

/// Streams a string representation of @p lane_position into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
std::ostream& operator<<(std::ostream& out, const LanePosition& lane_position);

/// Isometric velocity vector in a `Lane`-frame.
///
/// sigma_v, rho_v, and eta_v are the components of velocity in a
/// (sigma, rho, eta) coordinate system.  (sigma, rho, eta) have the same
/// orientation as the (s, r, h) at any given point in space, however they
/// form an isometric system with a Cartesian distance metric.  Hence,
/// IsoLaneVelocity represents a "real" physical velocity vector (albeit
/// with an orientation relative to the road surface).
struct IsoLaneVelocity {
  /// Default constructor.
  IsoLaneVelocity() = default;

  /// Fully parameterized constructor.
  IsoLaneVelocity(double _sigma_v, double _rho_v, double _eta_v) : sigma_v(_sigma_v), rho_v(_rho_v), eta_v(_eta_v) {}

  double sigma_v{};
  double rho_v{};
  double eta_v{};
};

/// A position in the road network, consisting of a pointer to a specific
/// Lane and a `Lane`-frame position in that Lane.
struct RoadPosition {
  /// Default constructor.
  RoadPosition() = default;

  /// Fully parameterized constructor.
  RoadPosition(const Lane* _lane, const LanePosition& _pos) : lane(_lane), pos(_pos) {}

  /// Indirection to #api::Lane::ToInertialPosition() method.
  /// @returns The InertialPosition that corresponds to `lane` and `pos`.
  /// @throws When member `lane` is nullptr.
  InertialPosition ToInertialPosition() const;

  const Lane* lane{};
  LanePosition pos;
};

/// Included in the return result of RoadGeometry::FindRoadPositions()
/// and RoadGeometry::ToRoadPosition().
struct RoadPositionResult {
  /// The candidate RoadPosition.
  RoadPosition road_position;
  /// The position that exactly corresponds to `road_position`.
  InertialPosition nearest_position;
  /// The distance between `nearest_position` and the `inertial_position` supplied
  /// to RoadGeometry::FindRoadPositions() or RoadGeometry::ToRoadPosition().
  double distance{};
};

/// Bounds in the lateral dimension (r component) of a `Lane`-frame, consisting
/// of a pair of minimum and maximum r value.  The bounds must straddle r = 0,
/// i.e., the minimum must be <= 0 and the maximum must be >= 0.
class RBounds {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RBounds)

  /// Default constructor.
  RBounds() = default;

  /// Fully parameterized constructor.
  /// @throws maliput::common::assertion_error When @p min is greater than 0.
  /// @throws maliput::common::assertion_error When @p max is smaller than 0.
  RBounds(double min, double max) : min_(min), max_(max) {
    MALIPUT_THROW_UNLESS(min <= 0.);
    MALIPUT_THROW_UNLESS(max >= 0.);
  }

  /// @name Getters and Setters
  //@{
  /// Gets minimum bound.
  double min() const { return min_; }
  /// Sets minimum bound.
  /// @throws maliput::common::assertion_error When @p min is greater than 0.
  void set_min(double min) {
    MALIPUT_THROW_UNLESS(min <= 0.);
    min_ = min;
  }
  /// Gets maximum bound.
  double max() const { return max_; }
  /// Sets maximum bound.
  /// @throws maliput::common::assertion_error When @p max is smaller than 0.
  void set_max(double max) {
    MALIPUT_THROW_UNLESS(max >= 0.);
    max_ = max;
  }
  //@}

 private:
  double min_{};
  double max_{};
};

/// Bounds in the elevation dimension (`h` component) of a `Lane`-frame,
/// consisting of a pair of minimum and maximum `h` value.  The bounds
/// must straddle `h = 0`, i.e., the minimum must be `<= 0` and the
/// maximum must be `>= 0`.
class HBounds {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(HBounds)

  /// Default constructor.
  HBounds() = default;

  /// Fully parameterized constructor.
  /// @throws maliput::common::assertion_error When @p min is greater than 0.
  /// @throws maliput::common::assertion_error When @p max is smaller than 0.
  HBounds(double min, double max) : min_(min), max_(max) {
    MALIPUT_THROW_UNLESS(min <= 0.);
    MALIPUT_THROW_UNLESS(max >= 0.);
  }

  /// @name Getters and Setters
  //@{
  /// Gets minimum bound.
  double min() const { return min_; }
  /// Sets minimum bound.
  /// @throws maliput::common::assertion_error When @p min is greater than 0.
  void set_min(double min) {
    MALIPUT_THROW_UNLESS(min <= 0.);
    min_ = min;
  }
  /// Gets maximum bound.
  double max() const { return max_; }
  /// Sets maximum bound.
  /// @throws maliput::common::assertion_error When @p max is smaller than 0.
  void set_max(double max) {
    MALIPUT_THROW_UNLESS(max >= 0.);
    max_ = max;
  }
  //@}

 private:
  double min_{};
  double max_{};
};

}  // namespace api
}  // namespace maliput
