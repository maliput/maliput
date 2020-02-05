#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "maliput/common/logger.h"
#include "maliput/math/matrix.h"

namespace maliput {
namespace math {
namespace test {

enum class CompareType { absolute, relative };

/**
 * Assert that two vectors @p v1 and @p v2 are equal down to a certain @p tolerance.
 *
 * @param v1 The first vector to compare.
 * @param v2 The second vector to compare.
 * @param tolerance The tolerance for determining equivalence.
 * @param compare_type Whether the tolerance is absolute or relative.
 * @return \::testing\::AssertionSuccess if the two matrices are equal based on the specified
 * tolerance, \::testing\::AssertionFailure otherwise.
 */
template <std::size_t N, typename Derived>
::testing::AssertionResult CompareVectors(const math::VectorBase<N, Derived>& v1,
                                          const math::VectorBase<N, Derived>& v2, double tolerance = 0.0,
                                          CompareType compare_type = CompareType::absolute) {
  for (int i = 0; i < N; i++) {
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

    if (compare_type == CompareType::absolute) {
      // Perform comparison using absolute tolerance.

      if (delta > tolerance) {
        return ::testing::AssertionFailure()
               << "Values at (" << i << ") exceed tolerance: " << v1[i] << " vs. " << v2[i] << ", diff = " << delta
               << ", tolerance = " << tolerance << "\nv1 =\n"
               << v1 << "\nv2 =\n"
               << v2 << "\ndelta=\n"
               << (v1 - v2);
      }
    } else {
      // Perform comparison using relative tolerance, see:
      // http://realtimecollisiondetection.net/blog/?p=89
      const auto max_value = std::max(std::abs(v1[i]), std::abs(v2[i]));
      const auto relative_tolerance = tolerance * std::max(decltype(max_value){1}, max_value);

      if (delta > relative_tolerance) {
        return ::testing::AssertionFailure()
               << "Values at (" << i << ") exceed tolerance: " << v1[i] << " vs. " << v2[i] << ", diff = " << delta
               << ", tolerance = " << tolerance << ", relative tolerance = " << relative_tolerance << "\nv1 =\n"
               << v1 << "\nv2 =\n"
               << v2 << "\ndelta=\n"
               << (v1 - v2);
      }
    }
  }

  return ::testing::AssertionSuccess() << "v1 =\n" << v1 << "\nis approximately equal to v2 =\n" << v2;
}

/**
 * Assert that two matrices @p m1 and @p m2 are equal down to a certain @p tolerance.
 *
 * @param m1 The first matrix to compare.
 * @param m2 The second matrix to compare.
 * @param tolerance The tolerance for determining equivalence.
 * @param compare_type Whether the tolerance is absolute or relative.
 * @return \::testing\::AssertionSuccess if the two matrices are equal based on the specified
 * tolerance, \::testing\::AssertionFailure otherwise.
 */
template <std::size_t N>
::testing::AssertionResult CompareMatrices(const math::Matrix<N>& m1, const math::Matrix<N>& m2, double tolerance = 0.0,
                                           CompareType compare_type = CompareType::absolute) {
  for (int ii = 0; ii < N; ii++) {
    for (int jj = 0; jj < N; jj++) {
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

      if (compare_type == CompareType::absolute) {
        // Perform comparison using absolute tolerance.

        if (delta > tolerance) {
          return ::testing::AssertionFailure()
                 << "Values at (" << ii << ", " << jj << ") exceed tolerance: " << m1[ii][jj] << " vs. " << m2[ii][jj]
                 << ", diff = " << delta << ", tolerance = " << tolerance << "\nm1 =\n"
                 << m1 << "\nm2 =\n"
                 << m2 << "\ndelta=\n"
                 << (m1 - m2);
        }
      } else {
        // Perform comparison using relative tolerance, see:
        // http://realtimecollisiondetection.net/blog/?p=89
        const auto max_value = std::max(std::abs(m1[ii][jj]), std::abs(m2[ii][jj]));
        const auto relative_tolerance = tolerance * std::max(decltype(max_value){1}, max_value);

        if (delta > relative_tolerance) {
          return ::testing::AssertionFailure()
                 << "Values at (" << ii << ", " << jj << ") exceed tolerance: " << m1[ii][jj] << " vs. " << m2[ii][jj]
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

}  // namespace test
}  // namespace math
}  // namespace maliput
