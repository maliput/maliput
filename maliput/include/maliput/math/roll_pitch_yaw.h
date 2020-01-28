#pragma once

#include "drake/math/roll_pitch_yaw.h"

namespace maliput {
namespace math {

/// @note Convenient alias to define the `maliput` rotation matrix based on
/// Euler angles (roll, pitch and yaw) type while the removal of `drake` out of
/// `maliput` core is performed. A custom implementation of `RollPitchYaw` will
/// be provided once the type migration is complete.
using RollPitchYaw = drake::math::RollPitchYaw<double>;

}  // namespace math
}  // namespace maliput
