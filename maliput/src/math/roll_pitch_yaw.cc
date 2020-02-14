/// @file roll_pitch_yaw.cc
/// Code in this file is inspired by:
/// https://github.com/RobotLocomotion/drake/blob/master/math/roll_pitch_yaw.cc
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

#include "maliput/math/roll_pitch_yaw.h"
#include <cmath>

namespace maliput {
namespace math {

RollPitchYaw::RollPitchYaw(const Vector3& rpy) { set(rpy); }

RollPitchYaw::RollPitchYaw(double roll, double pitch, double yaw) { set(roll, pitch, yaw); }

RollPitchYaw::RollPitchYaw(const Quaternion& quaternion) { SetFromQuaternion(quaternion); }

RollPitchYaw& RollPitchYaw::set(const Vector3& rpy) {
  roll_pitch_yaw_ = rpy;
  return *this;
}

RollPitchYaw& RollPitchYaw::set(double roll, double pitch, double yaw) {
  roll_pitch_yaw_.x() = roll;
  roll_pitch_yaw_.y() = pitch;
  roll_pitch_yaw_.z() = yaw;
  return *this;
}

void RollPitchYaw::SetFromQuaternion(const Quaternion& quaternion) {
  // This is copied from ignition::math::Quaternion::Euler()
  // TODO once a rotation matrix can be computed from a Quaternion,
  // copy from Drake CalcRollPitchYawFromQuaternionAndRotationMatrix
  Vector3 vec;
  using std::asin;
  using std::atan2;

  Quaternion copy = quaternion.normalized();
  double squ = copy.w() * copy.w();
  double sqx = copy.x() * copy.x();
  double sqy = copy.y() * copy.y();
  double sqz = copy.z() * copy.z();

  // Pitch
  double sarg = -2 * (copy.x() * copy.z() - copy.w() * copy.y());
  if (sarg <= -1.0) {
    vec.y() = -0.5 * M_PI;
  } else if (sarg >= 1.0) {
    vec.y() = 0.5 * M_PI;
  } else {
    vec.y() = asin(sarg);
  }

  // If the pitch angle is PI/2 or -PI/2, we can only compute
  // the sum roll + yaw.  However, any combination that gives
  // the right sum will produce the correct orientation, so we
  // set yaw = 0 and compute roll.
  // pitch angle is PI/2
  if (std::abs(sarg - 1) < kTolerance) {
    vec.z() = 0;
    vec.x() = atan2(2 * (copy.x() * copy.y() - copy.z() * copy.w()), squ - sqx + sqy - sqz);
  }
  // pitch angle is -PI/2
  else if (std::abs(sarg + 1) < kTolerance) {
    vec.z() = 0;
    vec.x() = atan2(-2 * (copy.x() * copy.y() - copy.z() * copy.w()), squ - sqx + sqy - sqz);
  } else {
    // Roll
    vec.x() = atan2(2 * (copy.y() * copy.z() + copy.w() * copy.x()), squ - sqx - sqy + sqz);

    // Yaw
    vec.z() = atan2(2 * (copy.x() * copy.y() + copy.w() * copy.z()), squ + sqx - sqy - sqz);
  }

  this->set(vec);
}

const Vector3& RollPitchYaw::vector() const { return roll_pitch_yaw_; }

Matrix3 RollPitchYaw::ToMatrix() const {
  const double& r = roll_pitch_yaw_.x();
  const double& p = roll_pitch_yaw_.y();
  const double& y = roll_pitch_yaw_.z();
  using std::cos;
  using std::sin;
  const double c0 = cos(r), c1 = cos(p), c2 = cos(y);
  const double s0 = sin(r), s1 = sin(p), s2 = sin(y);
  const double c2_s1 = c2 * s1, s2_s1 = s2 * s1;
  const double Rxx = c2 * c1;
  const double Rxy = c2_s1 * s0 - s2 * c0;
  const double Rxz = c2_s1 * c0 + s2 * s0;
  const double Ryx = s2 * c1;
  const double Ryy = s2_s1 * s0 + c2 * c0;
  const double Ryz = s2_s1 * c0 - c2 * s0;
  const double Rzx = -s1;
  const double Rzy = c1 * s0;
  const double Rzz = c1 * c0;
  return Matrix3({{Rxx, Rxy, Rxz}, {Ryx, Ryy, Ryz}, {Rzx, Rzy, Rzz}});
}

Quaternion RollPitchYaw::ToQuaternion() const {
  using std::cos;
  using std::sin;
  const double q0Half = roll_pitch_yaw_.x() / 2;
  const double q1Half = roll_pitch_yaw_.y() / 2;
  const double q2Half = roll_pitch_yaw_.z() / 2;
  const double c0 = cos(q0Half), s0 = sin(q0Half);
  const double c1 = cos(q1Half), s1 = sin(q1Half);
  const double c2 = cos(q2Half), s2 = sin(q2Half);
  const double c1_c2 = c1 * c2, s1_c2 = s1 * c2;
  const double s1_s2 = s1 * s2, c1_s2 = c1 * s2;
  const double w = c0 * c1_c2 + s0 * s1_s2;
  const double x = s0 * c1_c2 - c0 * s1_s2;
  const double y = c0 * s1_c2 + s0 * c1_s2;
  const double z = c0 * c1_s2 - s0 * s1_c2;
  return Quaternion(w, x, y, z);
}
}  // namespace math
}  // namespace maliput
