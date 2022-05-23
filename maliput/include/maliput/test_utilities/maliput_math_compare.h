// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
#pragma once

#include <gtest/gtest.h>

#include "maliput/math/matrix.h"

namespace maliput {
namespace math {
namespace test {

enum class CompareType { kAbsolute, kRelative };

/**
 * Assert that two vectors @p v1 and @p v2 are equal down to a certain @p tolerance.
 *
 * Instantiations for comparing Vector2, Vector3 and Vector4 are provided.
 *
 * @param v1 The first vector to compare.
 * @param v2 The second vector to compare.
 * @param tolerance The tolerance for determining equivalence.
 * @param compare_type Whether the tolerance is absolute or relative.
 * @return \::testing\::AssertionSuccess if the two matrices are equal based on the specified
 * tolerance, \::testing\::AssertionFailure otherwise.
 */
template <std::size_t N, typename Derived>
testing::AssertionResult CompareVectors(const math::VectorBase<N, Derived>& v1, const math::VectorBase<N, Derived>& v2,
                                        double tolerance = 0.0, CompareType compare_type = CompareType::kAbsolute);

/**
 * Assert that two matrices @p m1 and @p m2 are equal down to a certain @p tolerance.
 *
 * Instantiations for comparing Matrix2, Matrix3 and Matrix4 are provided.
 *
 * @param m1 The first matrix to compare.
 * @param m2 The second matrix to compare.
 * @param tolerance The tolerance for determining equivalence.
 * @param compare_type Whether the tolerance is absolute or relative.
 * @return \::testing\::AssertionSuccess if the two matrices are equal based on the specified
 * tolerance, \::testing\::AssertionFailure otherwise.
 */
template <std::size_t N>
testing::AssertionResult CompareMatrices(const math::Matrix<N>& m1, const math::Matrix<N>& m2, double tolerance = 0.0,
                                         CompareType compare_type = CompareType::kAbsolute);

}  // namespace test
}  // namespace math
}  // namespace maliput
