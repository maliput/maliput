#pragma once

#include "drake/common/eigen_types.h"

namespace maliput {
namespace math {

/// @note Convenient alias to define the `maliput` vector 3D type while the
/// removal of `drake` out of `maliput` core is performed. A custom
/// implementation of `Vector3` will be provided once the type migration is
/// complete.
using Vector2 = drake::Vector2<double>;
using Vector3 = drake::Vector3<double>;
using Vector4 = drake::Vector4<double>;

}  // namespace math
}  // namespace maliput
