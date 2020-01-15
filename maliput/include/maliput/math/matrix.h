#pragma once

#include "drake/common/eigen_types.h"

namespace maliput {
namespace math {

/// @note Convenient alias to define the `maliput` matrix 3x3 type while the
/// removal of `drake` out of `maliput` core is performed. A custom
/// implementation of `Matrix3` will be provided once the type migration is
/// complete.
using Matrix3 = drake::Matrix3<double>;

}  // namespace math
}  // namespace maliput
