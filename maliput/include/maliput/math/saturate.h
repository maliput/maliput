#pragma once

namespace maliput {
namespace math {

/// When `x` is less than `min`, it returns `min`. When `x` is greater than `max`, it returns `max`.
/// Otherwise, it returns `x`.
///
/// @throws common::assertion_error When `min` is greater than `max`.
double saturate(double x, double min, double max);

}  // namespace math
}  // namespace maliput
