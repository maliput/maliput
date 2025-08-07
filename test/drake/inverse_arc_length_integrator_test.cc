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
#include "maliput/drake/inverse_arc_length_integrator.h"

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

class InverseArcLengthIntegratorTest : public ::testing::Test {
 public:
  static constexpr double kTolerance{1e-6};
  const IntegratorConfiguration kZeroParameterConfiguration{
      // Lower bound of the function domain to be integrated.
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
  const IntegratorConfiguration kNonZeroParameterConfiguration{
      // Lower bound of the function domain to be integrated.
      1.0,
      // Lower bound, or constant, of the primitive (integral function).
      0.0,
      // Context, it works as a constant offset of the function parameter.
      math::Vector2{0., 0.},
      // Integrator target initial step size.
      1e-3,
      // Integrator maximum step size.
      1.0,
      // Integrator target accuracy.
      kTolerance};
};

TEST_F(InverseArcLengthIntegratorTest, StraightLineTest) {
  // Scalar ODE function that returns the inverse of the tangent vector norm of
  // a function.
  // In this case, it could be a straight line. A vector whose norm is 2.0 in
  // any direction multiplied by `p`.
  const auto scalar_ode_function = [](const double& p, const double& s, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(s);
    maliput::common::unused(k);
    return 1.0 / 2.0;
  };

  const math::Vector2 k{0.0, 0.0};
  const InverseArcLengthIntegrator dut(scalar_ode_function, kZeroParameterConfiguration);

  // Tests over Evaluate.
  EXPECT_NEAR(0.0, dut.Evaluate(1.0, k), kTolerance);
  EXPECT_NEAR(0.5, dut.Evaluate(2.0, k), kTolerance);
  EXPECT_NEAR(1.0, dut.Evaluate(3.0, k), kTolerance);
  EXPECT_NEAR(2.0, dut.Evaluate(5.0, k), kTolerance);

  // Tests over IntegralFunction
  const auto inverse_function = dut.InverseFunction(1.0 /* s0 */, 5.0 /* s1 */, k, kTolerance);

  EXPECT_NEAR(0.0, inverse_function(1.0), kTolerance);
  EXPECT_NEAR(0.5, inverse_function(2.0), kTolerance);
  EXPECT_NEAR(1.0, inverse_function(3.0), kTolerance);
  EXPECT_NEAR(2.0, inverse_function(5.0), kTolerance);
}

TEST_F(InverseArcLengthIntegratorTest, StraightLineTestWithNoneZeroParameterConfiguration) {
  // Scalar ODE function that returns the inverse of the tangent vector norm of
  // a function.
  // In this case, it could be a straight line. A vector whose norm is 2.0 in
  // any direction multiplied by `p`.
  const auto scalar_ode_function = [](const double& p, const double& s, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(s);
    maliput::common::unused(k);
    return 1.0 / 2.0;
  };

  const math::Vector2 k{0.0, 0.0};
  const InverseArcLengthIntegrator dut(scalar_ode_function, kNonZeroParameterConfiguration);

  // Tests over Evaluate.
  EXPECT_NEAR(1.0, dut.Evaluate(0.0, k), kTolerance);
  EXPECT_NEAR(1.5, dut.Evaluate(1.0, k), kTolerance);
  EXPECT_NEAR(2.0, dut.Evaluate(2.0, k), kTolerance);
  EXPECT_NEAR(3.0, dut.Evaluate(4.0, k), kTolerance);

  // Tests over IntegralFunction
  const auto inverse_function = dut.InverseFunction(0.0 /* s0 */, 4.0 /* s1 */, k, kTolerance);

  EXPECT_NEAR(1.0, inverse_function(0.0), kTolerance);
  EXPECT_NEAR(1.5, inverse_function(1.0), kTolerance);
  EXPECT_NEAR(2.0, inverse_function(2.0), kTolerance);
  EXPECT_NEAR(3.0, inverse_function(4.0), kTolerance);
}

TEST_F(InverseArcLengthIntegratorTest, ConstantArcInTheGroundTest) {
  // Scalar ODE function that returns always the inverse of the speed norm of
  // a function that covers a quarter circle when the independent variable `p`
  // goes from 0 to 1.
  const auto scalar_ode_function = [](const double& p, const double& s, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(s);
    maliput::common::unused(k);
    return 2. / M_PI;
  };

  const math::Vector2 k{0.0, 0.0};
  const InverseArcLengthIntegrator dut(scalar_ode_function, kZeroParameterConfiguration);

  // Tests over Evaluate.
  EXPECT_NEAR(0.0, dut.Evaluate(1.0, k), kTolerance);
  EXPECT_NEAR(0.5, dut.Evaluate(M_PI / 4.0 + 1.0, k), kTolerance);
  EXPECT_NEAR(1.0, dut.Evaluate(M_PI / 2.0 + 1.0, k), kTolerance);
  EXPECT_NEAR(2.0, dut.Evaluate(M_PI + 1.0, k), kTolerance);

  // Tests over IntegralFunction
  const auto integral_function = dut.InverseFunction(1.0 /* s0 */, M_PI + 1.0 /* s1 */, k, kTolerance);

  EXPECT_NEAR(0.0, integral_function(1.0), kTolerance);
  EXPECT_NEAR(0.5, integral_function(M_PI / 4.0 + 1.0), kTolerance);
  EXPECT_NEAR(1.0, integral_function(M_PI / 2.0 + 1.0), kTolerance);
  EXPECT_NEAR(2.0, integral_function(M_PI + 1.0), kTolerance);
}

TEST_F(InverseArcLengthIntegratorTest, ConstantArcInTheGroundTestWithNoneZeroParameterConfiguration) {
  // Scalar ODE function that returns always the inverse of the speed norm of
  // a function that covers a quarter circle when the independent variable `p`
  // goes from 0 to 1.
  const auto scalar_ode_function = [](const double& p, const double& s, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(s);
    maliput::common::unused(k);
    return 2. / M_PI;
  };

  const math::Vector2 k{0.0, 0.0};
  const InverseArcLengthIntegrator dut(scalar_ode_function, kNonZeroParameterConfiguration);

  // Tests over Evaluate.
  EXPECT_NEAR(1.0, dut.Evaluate(0.0, k), kTolerance);
  EXPECT_NEAR(1.5, dut.Evaluate(M_PI / 4.0, k), kTolerance);
  EXPECT_NEAR(2.0, dut.Evaluate(M_PI / 2.0, k), kTolerance);
  EXPECT_NEAR(3.0, dut.Evaluate(M_PI, k), kTolerance);

  // Tests over IntegralFunction
  const auto integral_function = dut.InverseFunction(0.0 /* s0 */, M_PI /* s1 */, k, kTolerance);

  EXPECT_NEAR(1.0, integral_function(0.0), kTolerance);
  EXPECT_NEAR(1.5, integral_function(M_PI / 4.0), kTolerance);
  EXPECT_NEAR(2.0, integral_function(M_PI / 2.0), kTolerance);
  EXPECT_NEAR(3.0, integral_function(M_PI), kTolerance);
}

TEST_F(InverseArcLengthIntegratorTest, ScalarOdeFunctionArgumentPassingEvaluation) {
  const math::Vector2 k{1.0, 2.0};

  // Scalar ODE function that returns the inverse of the tangent vector norm of
  // a function.
  // In this case, it could be a straight line. A vector whose norm is 2.0 in
  // any direction multiplied by `p`.
  const auto scalar_ode_function = [&expected_k = k](const double& p, const double& s,
                                                     const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(s);
    EXPECT_EQ(expected_k, k);
    return 1.0 / 2.0;
  };

  const InverseArcLengthIntegrator dut(scalar_ode_function, kZeroParameterConfiguration);
  const auto integral_function = dut.InverseFunction(1.0 /* s0 */, 5.0 /* s1 */, k, kTolerance);

  EXPECT_NEAR(0.5, dut.Evaluate(2.0, k), kTolerance);
  EXPECT_NEAR(0.5, integral_function(2.0), kTolerance);
}

TEST_F(InverseArcLengthIntegratorTest, ThrowsWhenIndependentVariableIsOutOfRange) {
  const math::Vector2 k{0., 0.};

  // Scalar ODE function that returns the inverse of the tangent vector norm of
  // a function.
  // In this case, it could be a straight line. A vector whose norm is 2.0 in
  // any direction multiplied by `p`.
  const auto scalar_ode_function = [](const double& p, const double& s, const math::Vector2& k) -> double {
    maliput::common::unused(p);
    maliput::common::unused(s);
    maliput::common::unused(k);
    return 1.0 / 2.0;
  };

  const InverseArcLengthIntegrator dut(scalar_ode_function, kZeroParameterConfiguration);
  const auto inverse_function = dut.InverseFunction(1.0 /* s0 */, 5.0 /* s1 */, k, kTolerance);

  EXPECT_THROW({ inverse_function(5.0 * kTolerance); }, maliput::common::assertion_error);
  EXPECT_THROW({ inverse_function(1.0 - 2.0 * kTolerance); }, maliput::common::assertion_error);
}

}  // namespace
}  // namespace test
}  // namespace drake
}  // namespace maliput
