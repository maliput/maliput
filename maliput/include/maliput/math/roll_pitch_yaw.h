#pragma once

#include "maliput/math/matrix.h"
#include "maliput/math/quaternion.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace math {

class RollPitchYaw {
 public:
  /// Constructs a RollPitchYaw with zeros for each angle.
  RollPitchYaw() = default;

  /// Constructs a RollPitchYaw from a 3x1 array of angles.
  /// @param[in] rpy 3x1 array with roll, pitch, yaw angles in radians.
  explicit RollPitchYaw(const Vector3& rpy);

  /// Constructs a RollPitchYaw from roll, pitch, yaw angles in radians.
  /// @param[in] roll x-directed angle in the SpaceXYZ rotation sequence.
  /// @param[in] pitch y-directed angle in the SpaceXYZ rotation sequence.
  /// @param[in] yaw z-directed angle in the SpaceXYZ rotation sequence.
  RollPitchYaw(double roll, double pitch, double yaw);

  /// Uses a Quaternion to construct a RollPitchYaw with
  /// roll-pitch-yaw angles `[r, p, y]` in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= n`.
  /// @param[in] quaternion unit Quaternion.
  explicit RollPitchYaw(const Quaternion& quaternion);

  /// Copy constructor.
  RollPitchYaw(const RollPitchYaw& other) = default;

  /// Move constructor.
  RollPitchYaw(RollPitchYaw&& other) = default;

  /// Assignment operator overload.
  /// @param other RollPitchYaw object.
  RollPitchYaw& operator=(const RollPitchYaw& other) = default;

  /// Move assignment operator overload.
  /// @param other RollPitchYaw object.
  RollPitchYaw& operator=(RollPitchYaw&& other) = default;

  /// Sets this RollPitchYaw from a 3x1 array of angles.
  /// @param[in] rpy 3x1 array with roll, pitch, yaw angles in radians.
  RollPitchYaw& set(const Vector3& rpy);

  /// Sets this RollPitchYaw from roll, pitch, yaw angles in radians.
  /// @param[in] roll x-directed angle in the SpaceXYZ rotation sequence.
  /// @param[in] pitch y-directed angle in the SpaceXYZ rotation sequence.
  /// @param[in] yaw z-directed angle in the SpaceXYZ rotation sequence.
  RollPitchYaw& set(double roll, double pitch, double yaw);

  /// Uses a Quaternion to set this RollPitchYaw with
  /// roll-pitch-yaw angles `[r, p, y]` in the range
  /// `-π <= r <= π`, `-π/2 <= p <= π/2`, `-π <= y <= n`.
  /// @param[in] quaternion unit Quaternion.
  void SetFromQuaternion(const Quaternion& quaternion);

  /// Returns the Vector3 underlying this RollPitchYaw.
  const Vector3& vector() const;

  /// Returns the roll-angle underlying this RollPitchYaw.
  double roll_angle() const;

  /// Returns the pitch-angle underlying this RollPitchYaw.
  double pitch_angle() const;

  /// Returns the yaw-angle underlying this RollPitchYaw.
  double yaw_angle() const;

  /// Sets the roll-angle underlying this RollPitchYaw.
  /// @param[in] roll roll angle in radians.
  void set_roll_angle(double r);

  /// Sets the pitch-angle underlying this RollPitchYaw.
  /// @param[in] pitch pitch angle in radians.
  void set_pitch_angle(double p);

  /// Sets the yaw-angle underlying this RollPitchYaw.
  /// @param[in] yaw yaw angle in radians.
  void set_yaw_angle(double y);

  /// Returns the 3x3 matrix representation of the rotation matrix that
  /// corresponds to this RollPitchYaw.
  Matrix3 ToMatrix() const;

  /// Returns a quaternion representation of this RollPitchYaw.
  Quaternion ToQuaternion() const;

 private:
  Vector3 roll_pitch_yaw_;
};

}  // namespace math
}  // namespace maliput
