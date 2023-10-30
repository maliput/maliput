// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/math/roll_pitch_yaw.cc
//
// Drake's license follows:
//
// All components of Drake are licensed under the BSD 3-Clause License
// shown below. Where noted in the source code, some portions may
// be subject to other permissive, non-viral licenses.
//
// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

#include "maliput/math/roll_pitch_yaw.h"

#include <cmath>

#include "maliput/common/maliput_throw.h"

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
  // based on Drake's CalcRollPitchYawFromQuaternionAndRotationMatrix
  // Uses a quaternion and its associated rotation matrix `R` to accurately
  // and efficiently calculate the roll-pitch-yaw angles (SpaceXYZ Euler angles)
  // that underlie `this` @RollPitchYaw, even when the pitch angle p is very
  // near a singularity (e.g., when p is within 1E-6 of π/2 or -π/2).
  // This algorithm was created October 2016 by Paul Mitiguy for TRI (Toyota).
  // More detail is in Chapter 6 Rotation Matrices II of the following:
  // "Advanced Dynamics and Motion Simulation, For professional engineers and scientists,"
  // Prodigy Press, Sunnyvale CA, 2017 (Paul Mitiguy).
  // Available at www.MotionGenesis.com
  using std::abs;
  using std::atan2;
  using std::sqrt;
  const Matrix3 R = quaternion.ToRotationMatrix();
  const Quaternion q = quaternion.normalized();
  Vector3 vec;

  // Calculate pitch using lots of information in the rotation matrix.
  // Rsum = abs( cos(pitch) ) is inherently non-negative.
  // R20 = -sin(pitch) may be negative, zero, or positive.
  const double R22 = R[2][2];
  const double R21 = R[2][1];
  const double R10 = R[1][0];
  const double R00 = R[0][0];
  const double Rsum = sqrt((R22 * R22 + R21 * R21 + R10 * R10 + R00 * R00) / 2);
  const double R20 = R[2][0];
  const double pitch = atan2(-R20, Rsum);

  // Calculate roll and yaw from Steps 2-6 (documented in drake's roll_pitch_yaw.cc:
  // https://github.com/RobotLocomotion/drake/blob/v0.15.0/math/roll_pitch_yaw.cc#L56-L95 ).
  const double e0 = q.w(), e1 = q.x();
  const double e2 = q.y(), e3 = q.z();
  const double yA = e1 + e3, xA = e0 - e2;
  const double yB = e3 - e1, xB = e0 + e2;
  const double epsilon = std::numeric_limits<double>::epsilon();
  const auto isSingularA = abs(yA) <= epsilon && abs(xA) <= epsilon;
  const auto isSingularB = abs(yB) <= epsilon && abs(xB) <= epsilon;
  const double zA = isSingularA ? double{0.0} : atan2(yA, xA);
  const double zB = isSingularB ? double{0.0} : atan2(yB, xB);
  double roll = zA - zB;  // First angle in rotation sequence.
  double yaw = zA + zB;   // Third angle in rotation sequence.

  // If necessary, modify angles roll and/or yaw to be between -pi and pi.
  if (roll > M_PI) roll = roll - 2 * M_PI;
  if (roll < -M_PI) roll = roll + 2 * M_PI;
  if (yaw > M_PI) yaw = yaw - 2 * M_PI;
  if (yaw < -M_PI) yaw = yaw + 2 * M_PI;

  // Return in Drake/ROS conventional SpaceXYZ roll, pitch, yaw (roll-pitch-yaw) order
  // (which is equivalent to BodyZYX yaw, pitch, roll order).
  this->set(roll, pitch, yaw);
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

Matrix3 RollPitchYaw::CalcRotationMatrixDt(const Vector3& rpyDt) const {
  // based on drake's CalcRotationMatrixDt
  // For the rotation matrix R generated by `this` RollPitchYaw, calculate the
  // partial derivatives of R with respect to roll `r`, pitch `p` and yaw `y`.
  Matrix3 R_r, R_p, R_y;  // ∂R/∂r, ∂R/∂p, ∂R/∂y
  CalcRotationMatrixDrDpDy(&R_r, &R_p, &R_y);

  // When rotation matrix R is regarded as an implicit function of t as
  // R(r(t), p(t), y(t)), the ordinary derivative of R with respect to t is
  // Ṙ = ∂R/∂r * ṙ + ∂R/∂p * ṗ + ∂R/∂y * ẏ
  const double rDt = rpyDt.x();
  const double pDt = rpyDt.y();
  const double yDt = rpyDt.z();
  return R_r * rDt + R_p * pDt + R_y * yDt;
}

void RollPitchYaw::CalcRotationMatrixDrDpDy(Matrix3* R_r, Matrix3* R_p, Matrix3* R_y) const {
  // based on drake's CalcRotationMatrixDrDpDy
  MALIPUT_THROW_UNLESS(R_r != nullptr && R_p != nullptr && R_y != nullptr);
  const double r = roll_angle();
  const double p = pitch_angle();
  const double y = yaw_angle();
  // clang-format off
  using std::sin;
  using std::cos;
  const double c0 = cos(r),  c1 = cos(p),  c2 = cos(y);
  const double s0 = sin(r),  s1 = sin(p),  s2 = sin(y);
  const double c2_s1 = c2 * s1, s2_s1 = s2 * s1, s2_s0 = s2 * s0, s2_c0 = s2 * c0;
  const double c2_c1 = c2 * c1, s2_c1 = s2 * c1, c2_s0 = c2 * s0, c2_c0 = c2 * c0;
  *R_r = Matrix3({0,   c2_s1 * c0 + s2_s0,   -c2_s1 * s0 + s2_c0,
                  0,   s2_s1 * c0 - c2_s0,   -s2_s1 * s0 - c2_c0,
                  0,              c1 * c0,              -c1 * s0});

  *R_p = Matrix3({-c2_s1,          c2_c1 * s0,            c2_c1 * c0,
                  -s2_s1,          s2_c1 * s0,            s2_c1 * c0,
                     -c1,            -s1 * s0,              -s1 * c0});

  *R_y = Matrix3({-s2_c1,  -s2_s1 * s0 - c2_c0,  -s2_s1 * c0 + c2_s0,
                   c2_c1,   c2_s1 * s0 - s2_c0,   c2_s1 * c0 + s2_s0,
                       0,                    0,                    0});
  // clang-format on
}
}  // namespace math
}  // namespace maliput
