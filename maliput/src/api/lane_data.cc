#include "maliput/api/lane_data.h"

#include <cmath>
#include <iostream>

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

std::ostream& operator<<(std::ostream& out, const GeoPosition& geo_position) {
  return out << "(x = " << geo_position.x() << ", y = " << geo_position.y() << ", z = " << geo_position.z() << ")";
}

std::ostream& operator<<(std::ostream& out, const LanePosition& lane_position) {
  return out << "(s = " << lane_position.s() << ", r = " << lane_position.r() << ", h = " << lane_position.h() << ")";
}

double GeoPosition::Distance(const GeoPosition& geo_position) const {
  return (this->xyz() - geo_position.xyz()).norm();
}

GeoPosition Rotation::Apply(const GeoPosition& geo_position) const {
  return GeoPosition::FromXyz(quaternion_ * geo_position.xyz());
}

math::Matrix3 Rotation::matrix() const { return quaternion_.ToRotationMatrix(); }

Rotation Rotation::Reverse() const {
  return Rotation::FromQuat(quaternion_ * math::Quaternion(M_PI, math::Vector3(0., 0., 1.)));
}

double Rotation::Distance(const Rotation& rot) const {
  // Compute transformed unit vectors of a frame A.
  const GeoPosition as = this->Apply({1., 0., 0.});
  const GeoPosition ar = this->Apply({0., 1., 0.});
  const GeoPosition ah = this->Apply({0., 0., 1.});
  // Compute transformed unit vectors of b frame B.
  const GeoPosition bs = rot.Apply({1., 0., 0.});
  const GeoPosition br = rot.Apply({0., 1., 0.});
  const GeoPosition bh = rot.Apply({0., 0., 1.});
  // Compute angles between pairs of unit vectors.
  const double ds = std::acos(as.xyz().dot(bs.xyz()));
  const double dr = std::acos(ar.xyz().dot(br.xyz()));
  const double dh = std::acos(ah.xyz().dot(bh.xyz()));

  return std::sqrt((ds * ds) + (dr * dr) + (dh * dh));
}

}  // namespace api
}  // namespace maliput
