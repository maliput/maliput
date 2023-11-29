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
#include "maliput/api/lane_data.h"

#include <cmath>
#include <iostream>

#include "maliput/api/lane.h"

namespace maliput {
namespace api {

std::ostream& operator<<(std::ostream& out, const LaneEnd::Which& which_end) {
  return out << (which_end == LaneEnd::kStart ? "start" : "finish");
}

std::ostream& operator<<(std::ostream& out, const Rotation& rotation) {
  const math::RollPitchYaw roll_pitch_yaw = rotation.rpy();
  return out << "(roll = " << roll_pitch_yaw.roll_angle() << ", pitch = " << roll_pitch_yaw.pitch_angle()
             << ", yaw = " << roll_pitch_yaw.yaw_angle() << ")";
}

std::ostream& operator<<(std::ostream& out, const InertialPosition& inertial_position) {
  return out << "(x = " << inertial_position.x() << ", y = " << inertial_position.y()
             << ", z = " << inertial_position.z() << ")";
}

std::ostream& operator<<(std::ostream& out, const LanePosition& lane_position) {
  return out << "(s = " << lane_position.s() << ", r = " << lane_position.r() << ", h = " << lane_position.h() << ")";
}

double InertialPosition::Distance(const InertialPosition& inertial_position) const {
  return (this->xyz() - inertial_position.xyz()).norm();
}

InertialPosition Rotation::Apply(const InertialPosition& inertial_position) const {
  return InertialPosition::FromXyz(quaternion_ * inertial_position.xyz());
}

InertialPosition RoadPosition::ToInertialPosition() const {
  MALIPUT_THROW_UNLESS(lane != nullptr);
  return lane->ToInertialPosition(pos);
};

math::Matrix3 Rotation::matrix() const { return quaternion_.ToRotationMatrix(); }

Rotation Rotation::Reverse() const {
  return Rotation::FromQuat(quaternion_ * math::Quaternion(M_PI, math::Vector3(0., 0., 1.)));
}

double Rotation::Distance(const Rotation& rot) const {
  // Compute transformed unit vectors of a frame A.
  const InertialPosition as = this->Apply({1., 0., 0.});
  const InertialPosition ar = this->Apply({0., 1., 0.});
  const InertialPosition ah = this->Apply({0., 0., 1.});
  // Compute transformed unit vectors of b frame B.
  const InertialPosition bs = rot.Apply({1., 0., 0.});
  const InertialPosition br = rot.Apply({0., 1., 0.});
  const InertialPosition bh = rot.Apply({0., 0., 1.});
  // Compute angles between pairs of unit vectors.
  const double ds = std::acos(as.xyz().dot(bs.xyz()));
  const double dr = std::acos(ar.xyz().dot(br.xyz()));
  const double dh = std::acos(ah.xyz().dot(bh.xyz()));

  return std::sqrt((ds * ds) + (dr * dr) + (dh * dh));
}

}  // namespace api
}  // namespace maliput
