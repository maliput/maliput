#include "maliput/math/roll_pitch_yaw.h"

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
}

const Vector3& RollPitchYaw::vector() const { return roll_pitch_yaw_; }

double RollPitchYaw::roll_angle() const { return roll_pitch_yaw_.x(); }

double RollPitchYaw::pitch_angle() const { return roll_pitch_yaw_.y(); }

double RollPitchYaw::yaw_angle() const { return roll_pitch_yaw_.z(); }

void RollPitchYaw::set_roll_angle(double r) { roll_pitch_yaw_.x() = r; }

void RollPitchYaw::set_pitch_angle(double p) { roll_pitch_yaw_.y() = p; }

void RollPitchYaw::set_yaw_angle(double y) { roll_pitch_yaw_.z() = y; }

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
