#pragma once

/// @file roll_pitch_yaw.h
/// Code in this file is inspired by:
/// https://github.com/RobotLocomotion/drake/blob/master/math/roll_pitch_yaw.h
///
/// Drake's license follows:
///
/// All components of Drake are licensed under the BSD 3-Clause License
/// shown below. Where noted in the source code, some portions may
/// be subject to other permissive, non-viral licenses.
///
/// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are
/// met:
///
/// Redistributions of source code must retain the above copyright notice,
/// this list of conditions and the following disclaimer.  Redistributions
/// in binary form must reproduce the above copyright notice, this list of
/// conditions and the following disclaimer in the documentation and/or
/// other materials provided with the distribution.  Neither the name of
/// the Massachusetts Institute of Technology nor the names of its
/// contributors may be used to endorse or promote products derived from
/// this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
/// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
/// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
/// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
/// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
/// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
/// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
/// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "maliput/common/maliput_copyable.h"
#include "maliput/math/matrix.h"
#include "maliput/math/quaternion.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace math {

/// This class represents the orientation between two arbitrary frames A and D
/// associated with a Space-fixed (extrinsic) X-Y-Z rotation by "roll-pitch-yaw"
/// angles `[r, p, y]`, which is equivalent to a Body-fixed (intrinsic) Z-Y-X
/// rotation by "yaw-pitch-roll" angles `[y, p, r]`.  The rotation matrix `R_AD`
/// associated with this roll-pitch-yaw `[r, p, y]` rotation sequence is equal
/// to the matrix multiplication shown below.
/// ```
///        ⎡cos(y) -sin(y)  0⎤   ⎡ cos(p)  0  sin(p)⎤   ⎡1      0        0 ⎤
/// R_AD = ⎢sin(y)  cos(y)  0⎥ * ⎢     0   1      0 ⎥ * ⎢0  cos(r)  -sin(r)⎥
///        ⎣    0       0   1⎦   ⎣-sin(p)  0  cos(p)⎦   ⎣0  sin(r)   cos(r)⎦
///      =       R_AB          *        R_BC          *        R_CD
/// ```
class RollPitchYaw {
 public:
  MALIPUT_DEFAULT_COPY_MOVE_ASSIGN(RollPitchYaw);

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
