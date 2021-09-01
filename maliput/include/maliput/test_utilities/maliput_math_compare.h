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
