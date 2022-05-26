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
#include "maliput/test_utilities/maliput_math_compare.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace maliput {
namespace math {
namespace test {

template <std::size_t N, typename Derived>
testing::AssertionResult CompareVectors(const math::VectorBase<N, Derived>& v1, const math::VectorBase<N, Derived>& v2,
                                        double tolerance, CompareType compare_type) {
  for (int i = 0; i < static_cast<int>(N); i++) {
    // First handle the corner cases of positive infinity, negative infinity,
    // and NaN
    const auto both_positive_infinity =
        v1[i] == std::numeric_limits<double>::infinity() && v2[i] == std::numeric_limits<double>::infinity();

    const auto both_negative_infinity =
        v1[i] == -std::numeric_limits<double>::infinity() && v2[i] == -std::numeric_limits<double>::infinity();

    const auto both_nan = std::isnan(v1[i]) && std::isnan(v2[i]);

    if (both_positive_infinity || both_negative_infinity || both_nan) continue;

    // Check for case where one value is NaN and the other is not
    if ((std::isnan(v1[i]) && !std::isnan(v2[i])) || (!std::isnan(v1[i]) && std::isnan(v2[i]))) {
      return ::testing::AssertionFailure() << "NaN mismatch at (" << i << "):\nv1 =\n" << v1 << "\nv2 =\n" << v2;
    }

    // Determine whether the difference between the two vectors is less than
    // the tolerance.
    const auto delta = std::abs(v1[i] - v2[i]);

    if (compare_type == CompareType::kAbsolute) {
      // Perform comparison using absolute tolerance.

      if (delta > tolerance) {
        return ::testing::AssertionFailure()
               << "Value at (" << i << ") exceeds tolerance: " << v1[i] << " vs. " << v2[i] << ", diff = " << delta
               << ", tolerance = " << tolerance << "\nv1 =\n"
               << v1 << "\nv2 =\n"
               << v2 << "\ndelta=\n"
               << (v1 - v2);
      }
    } else {
      // Perform comparison using relative tolerance, see:
      // http://realtimecollisiondetection.net/blog/?p=89
      const auto max_value = std::max(std::abs(v1[i]), std::abs(v2[i]));
      const auto relative_tolerance = tolerance * std::max(1., max_value);

      if (delta > relative_tolerance) {
        return ::testing::AssertionFailure()
               << "Value at (" << i << ") exceeds tolerance: " << v1[i] << " vs. " << v2[i] << ", diff = " << delta
               << ", tolerance = " << tolerance << ", relative tolerance = " << relative_tolerance << "\nv1 =\n"
               << v1 << "\nv2 =\n"
               << v2 << "\ndelta=\n"
               << (v1 - v2);
      }
    }
  }

  return ::testing::AssertionSuccess() << "v1 =\n" << v1 << "\nis approximately equal to v2 =\n" << v2;
}

template <std::size_t N>
testing::AssertionResult CompareMatrices(const math::Matrix<N>& m1, const math::Matrix<N>& m2, double tolerance,
                                         CompareType compare_type) {
  for (int ii = 0; ii < static_cast<int>(N); ii++) {
    for (int jj = 0; jj < static_cast<int>(N); jj++) {
      // First handle the corner cases of positive infinity, negative infinity,
      // and NaN
      const auto both_positive_infinity = m1[ii][jj] == std::numeric_limits<double>::infinity() &&
                                          m2[ii][jj] == std::numeric_limits<double>::infinity();

      const auto both_negative_infinity = m1[ii][jj] == -std::numeric_limits<double>::infinity() &&
                                          m2[ii][jj] == -std::numeric_limits<double>::infinity();

      const auto both_nan = std::isnan(m1[ii][jj]) && std::isnan(m2[ii][jj]);

      if (both_positive_infinity || both_negative_infinity || both_nan) continue;

      // Check for case where one value is NaN and the other is not
      if ((std::isnan(m1[ii][jj]) && !std::isnan(m2[ii][jj])) || (!std::isnan(m1[ii][jj]) && std::isnan(m2[ii][jj]))) {
        return ::testing::AssertionFailure() << "NaN mismatch at (" << ii << ", " << jj << "):\nm1 =\n"
                                             << m1 << "\nm2 =\n"
                                             << m2;
      }

      // Determine whether the difference between the two matrices is less than
      // the tolerance.
      const auto delta = std::abs(m1[ii][jj] - m2[ii][jj]);

      if (compare_type == CompareType::kAbsolute) {
        // Perform comparison using absolute tolerance.

        if (delta > tolerance) {
          return ::testing::AssertionFailure()
                 << "Value at (" << ii << ", " << jj << ") exceeds tolerance: " << m1[ii][jj] << " vs. " << m2[ii][jj]
                 << ", diff = " << delta << ", tolerance = " << tolerance << "\nm1 =\n"
                 << m1 << "\nm2 =\n"
                 << m2 << "\ndelta=\n"
                 << (m1 - m2);
        }
      } else {
        // Perform comparison using relative tolerance, see:
        // http://realtimecollisiondetection.net/blog/?p=89
        const auto max_value = std::max(std::abs(m1[ii][jj]), std::abs(m2[ii][jj]));
        const auto relative_tolerance = tolerance * std::max(1., max_value);

        if (delta > relative_tolerance) {
          return ::testing::AssertionFailure()
                 << "Value at (" << ii << ", " << jj << ") exceeds tolerance: " << m1[ii][jj] << " vs. " << m2[ii][jj]
                 << ", diff = " << delta << ", tolerance = " << tolerance
                 << ", relative tolerance = " << relative_tolerance << "\nm1 =\n"
                 << m1 << "\nm2 =\n"
                 << m2 << "\ndelta=\n"
                 << (m1 - m2);
        }
      }
    }
  }

  return ::testing::AssertionSuccess() << "m1 =\n" << m1 << "\nis approximately equal to m2 =\n" << m2;
}

//! @cond Doxygen_Suppress
// Explicit instanciations
template ::testing::AssertionResult CompareVectors(const VectorBase<2, Vector2>&, const VectorBase<2, Vector2>&, double,
                                                   CompareType);
template ::testing::AssertionResult CompareVectors(const VectorBase<3, Vector3>&, const VectorBase<3, Vector3>&, double,
                                                   CompareType);
template ::testing::AssertionResult CompareVectors(const VectorBase<4, Vector4>&, const VectorBase<4, Vector4>&, double,
                                                   CompareType);
template ::testing::AssertionResult CompareMatrices(const Matrix<2>&, const Matrix<2>&, double, CompareType);
template ::testing::AssertionResult CompareMatrices(const Matrix<3>&, const Matrix<3>&, double, CompareType);
template ::testing::AssertionResult CompareMatrices(const Matrix<4>&, const Matrix<4>&, double, CompareType);
//! @endcond

}  // namespace test
}  // namespace math
}  // namespace maliput
