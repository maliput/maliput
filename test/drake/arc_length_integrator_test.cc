// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota.
// All rights reserved.
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
#include "maliput/drake/arc_length_integrator.h"

#include <cmath>
#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "maliput/common/maliput_error.h"
#include "maliput/common/maliput_unused.h"
#include "maliput/drake/arc_length_integrator.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace drake {
namespace test {
namespace {

class ArcLengthIntegratorTest : public ::testing::Test {
 public:
  static constexpr double kTolerance{1e-6};
  const IntegratorConfiguration kConfiguration{// Lower bound of the function domain to be integrated.
                                               0.0,
                                               // Lower bound, or constant, of the primitive (integral function).
                                               1.0,
                                               // Context, it works as a constant offset of the function parameter.
                                               math::Vector2{0., 0.},
                                               // Integrator target initial step size.
                                               1e-3,
                                               // Integrator maximum step size.
                                               1.0,
                                               // Integrator target accuracy.
                                               kTolerance};
};

TEST_F(ArcLengthIntegratorTest, StraightLineTest) {
  // Integrable function that returns the tangent vector norm of a function.
  // In this case, it could be a straight line. A vector whose norm is 2.0 in
  // any direction multiplied by `p`.
  const auto integrable_function = [](const double& p, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(k);
    return 2.0;
  };

  const math::Vector2 k{0.0, 0.0};
  const ArcLengthIntegrator dut(integrable_function, kConfiguration);

  // Tests over Evaluate.
  EXPECT_NEAR(0.0, dut.Evaluate(0.0, k), kTolerance);
  EXPECT_NEAR(1.0, dut.Evaluate(0.5, k), kTolerance);
  EXPECT_NEAR(2.0, dut.Evaluate(1.0, k), kTolerance);
  EXPECT_NEAR(4.0, dut.Evaluate(2.0, k), kTolerance);

  // Tests over IntegralFunction
  const auto integral_function = dut.IntegralFunction(0.0 /* p0 */, 2.0 /* p1 */, k, kTolerance);

  EXPECT_NEAR(0.0, integral_function(0.0), kTolerance);
  EXPECT_NEAR(1.0, integral_function(0.5), kTolerance);
  EXPECT_NEAR(2.0, integral_function(1.0), kTolerance);
  EXPECT_NEAR(4.0, integral_function(2.0), kTolerance);
}

TEST_F(ArcLengthIntegratorTest, ConstantArcInTheGroundTest) {
  // Integrable function that returns always the speed norm of a function that
  // covers a quarter circle when the independent variable `p` goes from 0 to 1.
  const auto integrable_function = [](const double& p, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(k);
    return M_PI / 2.;
  };

  const math::Vector2 k{0.0, 0.0};
  const ArcLengthIntegrator dut(integrable_function, kConfiguration);

  // Tests over Evaluate.
  EXPECT_NEAR(0.0, dut.Evaluate(0.0, k), kTolerance);
  EXPECT_NEAR(M_PI / 4.0, dut.Evaluate(0.5, k), kTolerance);
  EXPECT_NEAR(M_PI / 2.0, dut.Evaluate(1.0, k), kTolerance);
  EXPECT_NEAR(M_PI, dut.Evaluate(2.0, k), kTolerance);

  // Tests over IntegralFunction
  const auto integral_function = dut.IntegralFunction(0.0 /* p0 */, 2.0 /* p1 */, k, kTolerance);

  EXPECT_NEAR(0, integral_function(0.0), kTolerance);
  EXPECT_NEAR(M_PI / 4.0, integral_function(0.5), kTolerance);
  EXPECT_NEAR(M_PI / 2.0, integral_function(1.0), kTolerance);
  EXPECT_NEAR(M_PI, integral_function(2.0), kTolerance);
}

TEST_F(ArcLengthIntegratorTest, IntegralFunctionArgumentPassingEvaluation) {
  const math::Vector2 k{1.0, 2.0};

  // Integrable function that returns the tangent vector norm of a function.
  // In this case, it could be a straight line. A vector whose norm is 2.0 in
  // any direction multiplied by `p`.
  const auto integrable_function = [&expected_k = k](const double& p, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    EXPECT_EQ(expected_k, k);
    return 2.0;
  };

  const ArcLengthIntegrator dut(integrable_function, kConfiguration);
  const auto integral_function = dut.IntegralFunction(0.0 /* p0 */, 2.0 /* p1 */, k, kTolerance);

  EXPECT_NEAR(1.0, dut.Evaluate(0.5, k), kTolerance);
  EXPECT_NEAR(1.0, integral_function(0.5), kTolerance);
}

TEST_F(ArcLengthIntegratorTest, ThrowsWhenIndependentVariableIsOutOfRange) {
  const math::Vector2 k{0., 0.};

  // Integrable function that returns the tangent vector norm of a function.
  // In this case, it could be a straight line. A vector whose norm is 2.0 in
  // any direction multiplied by `p`.
  const auto integrable_function = [](const double& p, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(k);
    return 2.0;
  };

  const ArcLengthIntegrator dut(integrable_function, kConfiguration);
  const auto integral_function = dut.IntegralFunction(0.0 /* p0 */, 2.0 /* p1 */, k, kTolerance);

  EXPECT_THROW({ integral_function(2.0 + 2.0 * kTolerance); }, maliput::common::assertion_error);
  EXPECT_THROW({ integral_function(-2.0 * kTolerance); }, maliput::common::assertion_error);
}

}  // namespace
}  // namespace test
}  // namespace drake
}  // namespace maliput
