// BSD 3-Clause License
//
// Copyright (c) 2024-2026, Woven by Toyota. All rights reserved.
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
#include "maliput/math/fresnel.h"

#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "assert_compare.h"
#include "maliput/math/compare.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace math {
namespace test {
namespace {

using maliput::test::AssertCompare;

// The following code can be validated against python's scipy.special.fresnel function.
// @code{.py}
// >>> from scipy import special as sc
// >>> for t in [0.0, 0.5, 1.0, 2.0, 10., 100000000.]:
// ...   print(sc.fresnel(t))
// ...
// (0.0, 0.0)
// (0.06473243285999929, 0.4923442258714464)
// (0.4382591473903547, 0.779893400376823)
// (0.34341567836369824, 0.48825340607534073)
// (0.46816997858488224, 0.49989869420551575)
// (0.50000000185342, 0.49999999741215284)
// >>> for t in [0.0, 0.5, 1.0, 2.0, 100000000.]:
// ...   print(sc.fresnel(-t))
// ...
// (0.0, 0.0)
// (-0.06473243285999929, -0.4923442258714464)
// (-0.4382591473903547, -0.779893400376823)
// (-0.34341567836369824, -0.48825340607534073)
// (-0.46816997858488224, -0.49989869420551575)
// (-0.50000000185342, -0.49999999741215284)
// @endcode{.py}
//
// Note: the python function returns S,C whereas we return C, S.

// Used to hold the input argument of the Fresnel equation and the expected result shown above.
struct FresnelTestConfig {
  double t{};
  Vector2 expected_result;
};

GTEST_TEST(FresnelCosineAndSineTest, ValidateValues) {
  constexpr double kTolerance{5e-8};  // Empirically found.
  const std::array<FresnelTestConfig, /*11*/ 13> kValuesUnderTest{
      FresnelTestConfig{0., Vector2{0.0, 0.0}},
      // This value has been set in particular to make sure we are covering all the
      // ranges in the implementation with a value that was meaningful in backend
      // development. The same applies to the the negative value.
      FresnelTestConfig{0.282095, Vector2{0.281654543994548, 0.011740863890491865}},
      FresnelTestConfig{0.5, Vector2{0.4923442258714464, 0.06473243285999929}},
      FresnelTestConfig{1.0, Vector2{0.779893400376823, 0.4382591473903547}},
      FresnelTestConfig{2.0, Vector2{0.48825340607534073, 0.34341567836369824}},
      FresnelTestConfig{10.0, Vector2{0.49989869420551575, 0.46816997858488224}},
      FresnelTestConfig{100000000.0, Vector2{0.49999999741215284, 0.50000000185342}},
      // Negative values are in the 3rd quadrant.
      FresnelTestConfig{-0.282095, Vector2{-0.281654543994548, -0.011740863890491865}},
      FresnelTestConfig{-0.5, Vector2{-0.4923442258714464, -0.06473243285999929}},
      FresnelTestConfig{-1.0, Vector2{-0.779893400376823, -0.4382591473903547}},
      FresnelTestConfig{-2.0, Vector2{-0.48825340607534073, -0.34341567836369824}},
      FresnelTestConfig{-10.0, Vector2{-0.49989869420551575, -0.46816997858488224}},
      FresnelTestConfig{-100000000.0, Vector2{-0.49999999741215284, -0.50000000185342}},
  };

  for (const FresnelTestConfig& vut /* value under test */ : kValuesUnderTest) {
    EXPECT_TRUE(AssertCompare(CompareVectors(vut.expected_result, ComputeFresnelCosineAndSine(vut.t), kTolerance)))
        << "Failed with t: " << vut.t;
  }
}

GTEST_TEST(FresnelSpiralHeadingTest, ValidateValues) {
  constexpr double kTolerance{1e-12};

  EXPECT_NEAR(0., FresnelSpiralHeading(0. /* t */, 1. /* k_dot */), kTolerance);
  EXPECT_NEAR(0., FresnelSpiralHeading(1. /* t */, 0. /* k_dot */), kTolerance);
  EXPECT_NEAR(0.5, FresnelSpiralHeading(1. /* t */, 1. /* k_dot */), kTolerance);
  EXPECT_NEAR(1.0, FresnelSpiralHeading(1. /* t */, 2. /* k_dot */), kTolerance);
  EXPECT_NEAR(2.0, FresnelSpiralHeading(2. /* t */, 1. /* k_dot */), kTolerance);
  EXPECT_NEAR(1.6875, FresnelSpiralHeading(1.5 /* t */, 1.5 /* k_dot */), kTolerance);
  EXPECT_NEAR(12.5, FresnelSpiralHeading(5. /* t */, 1. /* k_dot */), kTolerance);
}

GTEST_TEST(FresnelSpiralHeadingDotTest, ValidateValues) {
  constexpr double kTolerance{1e-12};

  EXPECT_NEAR(0., FresnelSpiralHeadingDot(0. /* t */, 1. /* k_dot */), kTolerance);
  EXPECT_NEAR(0., FresnelSpiralHeadingDot(1. /* t */, 0. /* k_dot */), kTolerance);
  EXPECT_NEAR(1.0, FresnelSpiralHeadingDot(1. /* t */, 1. /* k_dot */), kTolerance);
  EXPECT_NEAR(2.0, FresnelSpiralHeadingDot(1. /* t */, 2. /* k_dot */), kTolerance);
  EXPECT_NEAR(2.0, FresnelSpiralHeadingDot(2. /* t */, 1. /* k_dot */), kTolerance);
  EXPECT_NEAR(2.25, FresnelSpiralHeadingDot(1.5 /* t */, 1.5 /* k_dot */), kTolerance);
  EXPECT_NEAR(5.0, FresnelSpiralHeadingDot(5. /* t */, 1. /* k_dot */), kTolerance);
}

}  // namespace
}  // namespace test
}  // namespace math
}  // namespace maliput
