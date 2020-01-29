#include "maliput/math/saturate.h"

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace math {

double saturate(double x, double min, double max) {
  MALIPUT_THROW_UNLESS(min <= max);
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

}  // namespace math
}  // namespace maliput
