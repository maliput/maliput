// BSD 3-Clause License
//
// Copyright (c) 2023-2026, Woven by Toyota.
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

#include <Eigen/Dense>
#include <maliput/common/logger.h>
#include <maliput/common/maliput_throw.h>
#include <maliput/drake/common/eigen_types.h>
#include <maliput/drake/systems/analysis/integrator_base.h>
#include <maliput/drake/systems/analysis/scalar_dense_output.h>
#include <maliput/drake/systems/analysis/scalar_initial_value_problem.h>
#include <maliput/math/saturate.h>
#include <maliput/math/vector.h>

namespace maliput {
namespace drake {
namespace {

struct InverseArcLengthIntegratorFunctionWrapper {
  explicit InverseArcLengthIntegratorFunctionWrapper(const InverseArcLengthIntegrator::ScalarOdeFunction& f_in)
      : f(f_in) {}

  double operator()(const double& s, const double& p, const VectorX<double>& k) const {
    const math::Vector2 math_k(k(0), k(1));
    return f(s, p, math_k);
  }

  InverseArcLengthIntegrator::ScalarOdeFunction f;
};

}  // namespace

struct InverseArcLengthIntegrator::InverseArcLengthIntegratorDataImpl {
  std::unique_ptr<systems::ScalarInitialValueProblem<double>> scalar_initial_value_problem{};
};

InverseArcLengthIntegrator::InverseArcLengthIntegrator(const InverseArcLengthIntegrator::ScalarOdeFunction& function,
                                                       const IntegratorConfiguration& config) {
  impl_ = std::make_unique<InverseArcLengthIntegratorDataImpl>();

  // Construct the ScalarInitialValueProblem.
  const VectorX<double> default_parameters = (VectorX<double>(2) << config.k[0], config.k[1]).finished();
  const systems::ScalarInitialValueProblem<double>::ScalarOdeContext scalar_initial_value_problem_values(
      config.image_lower_bound, config.parameter_lower_bound, default_parameters);
  impl_->scalar_initial_value_problem = std::make_unique<systems::ScalarInitialValueProblem<double>>(
      InverseArcLengthIntegratorFunctionWrapper(function), scalar_initial_value_problem_values);
  // Configure the integrator.
  systems::IntegratorBase<double>& integrator = impl_->scalar_initial_value_problem->get_mutable_integrator();
  integrator.request_initial_step_size_target(config.initial_step_size_target);
  integrator.set_maximum_step_size(config.maximum_step_size);
  integrator.set_target_accuracy(config.target_accuracy);
}

double InverseArcLengthIntegrator::Evaluate(double s, const math::Vector2& k) const {
  // Populates parameter vector with (r, h) coordinate values.
  systems::ScalarInitialValueProblem<double>::ScalarOdeContext context;
  context.k = (VectorX<double>(2) << k[0], k[1]).finished();
  return impl_->scalar_initial_value_problem->Solve(s, context);
}

InverseArcLengthIntegrator::~InverseArcLengthIntegrator() = default;

std::function<double(double)> InverseArcLengthIntegrator::InverseFunction(double s0, double s1, const math::Vector2& k,
                                                                          double tolerance, double epsilon) const {
  // Populates parameter vector with (r, h) coordinate values.
  systems::ScalarInitialValueProblem<double>::ScalarOdeContext context;
  context.k = (VectorX<double>(2) << k[0], k[1]).finished();
  // Prepares dense output for shared ownership, as std::function
  // instances only take copyable callables.
  const std::shared_ptr<systems::ScalarDenseOutput<double>> dense_output{
      impl_->scalar_initial_value_problem->DenseSolve(s1, context)};
  MALIPUT_THROW_UNLESS(dense_output->start_time() <= s0);
  MALIPUT_THROW_UNLESS(dense_output->end_time() >= s1 - epsilon);
  return [dense_output, tolerance, s0, s1](double s) -> double {
    // Saturates s to lie within the [s0, s1] interval.
    const double saturated_s = maliput::math::saturate(s, s0, s1);
    MALIPUT_THROW_UNLESS(std::abs(saturated_s - s) < tolerance);
    return dense_output->EvaluateScalar(saturated_s);
  };
}

}  // namespace drake
}  // namespace maliput
