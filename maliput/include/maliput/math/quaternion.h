#pragma once

#include "drake/common/eigen_types.h"

namespace maliput {
namespace math {

/// @note Convenient alias to define the `maliput` quaternion type while the
/// removal of `drake` out of `maliput` core is performed. A custom
/// implementation of `Quaternion` will be provided once the type migration is
/// complete.
using Quaternion = drake::Quaternion<double>;

}  // namespace math
}  // namespace maliput
