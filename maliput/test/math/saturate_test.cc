#include "maliput/math/saturate.h"

#include <limits>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace math {
namespace test {
namespace {

constexpr double kMin{0.};
constexpr double kMax{10.};

GTEST_TEST(SaturateTest, ArgumentCheks) {
  EXPECT_THROW(saturate(kMin, kMax, kMin), common::assertion_error);
  EXPECT_NO_THROW(saturate(kMin, kMin, kMax));
}

GTEST_TEST(SaturateTest, InRange) {
  double x = (kMin + kMax) / 2.;
  EXPECT_EQ(saturate(x, kMin, kMax), x);

  x = kMin;
  EXPECT_EQ(saturate(x, kMin, kMax), x);

  x = kMax;
  EXPECT_EQ(saturate(x, kMin, kMax), x);
}

GTEST_TEST(SaturateTest, SaturateToMinimum) {
  double x = -kMax;
  EXPECT_EQ(saturate(x, kMin, kMax), kMin);

  x = kMin - std::numeric_limits<double>::epsilon();
  EXPECT_EQ(saturate(x, kMin, kMax), kMin);
}

GTEST_TEST(SaturateTest, SaturateToMaximum) {
  double x = 2. * kMax;
  EXPECT_EQ(saturate(x, kMin, kMax), kMax);

  x = kMax + std::numeric_limits<double>::epsilon();
  EXPECT_EQ(saturate(x, kMin, kMax), kMax);
}

}  // namespace
}  // namespace test
}  // namespace math
}  // namespace maliput
