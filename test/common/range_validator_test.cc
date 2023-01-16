// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2020-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput/common/range_validator.h"

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace common {
namespace test {

class RangeValidatorConstructorValidation : public ::testing::Test {
 protected:
  void SetUp() override {}

  static constexpr double kMin{0.5};
  static constexpr double kMax{1034.};
  static constexpr double kTolerance{1e-3};
  static constexpr double kEpsilon{1e-5};
};

// no throw
TEST_F(RangeValidatorConstructorValidation, WellConstructed) {
  EXPECT_NO_THROW({ RangeValidator::GetAbsoluteEpsilonValidator(kMin, kMax, kTolerance, kEpsilon); });
}

// relative epsilon > tolerance
// kEpsilon * (kMax - kMin) > tolerance
TEST_F(RangeValidatorConstructorValidation, RelativeEpsilonGreaterThanTolerance) {
  EXPECT_THROW({ RangeValidator::GetRelativeEpsilonValidator(kMin, kMax, kTolerance, kEpsilon); },
               maliput::common::assertion_error);
}

// min > max
TEST_F(RangeValidatorConstructorValidation, MinGreaterThanMax) {
  EXPECT_THROW({ RangeValidator::GetAbsoluteEpsilonValidator(kMax, kMin, kTolerance, kEpsilon); },
               maliput::common::assertion_error);
}

// epsilon > tolerance
TEST_F(RangeValidatorConstructorValidation, EpsilonGreaterThanTolerance) {
  EXPECT_THROW({ RangeValidator::GetAbsoluteEpsilonValidator(kMin, kMax, kEpsilon, kTolerance); },
               maliput::common::assertion_error);
}

// min + epsilon > max (applies the other way around, it's specially provided
// in constructor code to validate numerical error).
TEST_F(RangeValidatorConstructorValidation, MinPlusEpsilonGreaterThanMax) {
  EXPECT_THROW({ RangeValidator::GetAbsoluteEpsilonValidator(kMin, kMax, 2 * kMax, kMax); },
               maliput::common::assertion_error);
}

class RangeValidatorAbsoluteEpsilonRange : public ::testing::Test {
 protected:
  void SetUp() override {}

  static constexpr double kMin{0.5};
  static constexpr double kMax{3.};
  static constexpr double kTolerance{1e-3};
  static constexpr double kEpsilon{1e-5};
  const RangeValidator dut{RangeValidator::GetAbsoluteEpsilonValidator(kMin, kMax, kTolerance, kEpsilon)};
};

// In the middle of the range.
TEST_F(RangeValidatorAbsoluteEpsilonRange, MiddleOfRange) {
  const double kS{2.};
  EXPECT_DOUBLE_EQ(dut(kS), kS);
}

// In the maximum of the range.
TEST_F(RangeValidatorAbsoluteEpsilonRange, MaxLimitOfRange) {
  const double kS{kMax};
  EXPECT_DOUBLE_EQ(dut(kS), kS - kEpsilon);
}

// In the minimum of the range.
TEST_F(RangeValidatorAbsoluteEpsilonRange, MinLimitOfRange) {
  const double kS{kMin};
  EXPECT_DOUBLE_EQ(dut(kS), kS + kEpsilon);
}

// Exceeding the maximum but within linear tolerance.
TEST_F(RangeValidatorAbsoluteEpsilonRange, ExceedsMaximum) {
  const double kS{kMax + kTolerance / 2.};
  EXPECT_DOUBLE_EQ(dut(kS), kMax - kEpsilon);
}

// Exceeding the minimum but within linear tolerance.
TEST_F(RangeValidatorAbsoluteEpsilonRange, ExceedsMinimum) {
  const double kS{kMin - kTolerance / 2.};
  EXPECT_DOUBLE_EQ(dut(kS), kMin + kEpsilon);
}

// Expects throw because of out of bounds.
TEST_F(RangeValidatorAbsoluteEpsilonRange, OutOfBounds) {
  const double kS{kMax + 10 * kTolerance};
  EXPECT_THROW({ dut(kS); }, maliput::common::assertion_error);
}

class RangeValidatorRelativeEpsilonRange : public ::testing::Test {
 protected:
  void SetUp() override {}

  static constexpr double kMin{0.5};
  static constexpr double kMax{100.5};
  static constexpr double kRange{kMax - kMin};
  static constexpr double kTolerance{1e-3};
  static constexpr double kEpsilon{1e-8};
  static constexpr double kRelativeEpsilon{kEpsilon * kRange};
  const RangeValidator dut{RangeValidator::GetRelativeEpsilonValidator(kMin, kMax, kTolerance, kEpsilon)};
};

// In the middle of the range.
TEST_F(RangeValidatorRelativeEpsilonRange, MiddleOfRange) {
  const double kS{kMax - kMin};
  EXPECT_DOUBLE_EQ(dut(kS), kS);
}

// In the maximum of the range.
TEST_F(RangeValidatorRelativeEpsilonRange, MaxLimitOfRange) {
  const double kS{kMax};
  EXPECT_DOUBLE_EQ(dut(kS), kS - kRelativeEpsilon);
}

// In the minimum of the range.
TEST_F(RangeValidatorRelativeEpsilonRange, MinLimitOfRange) {
  const double kS{kMin};
  EXPECT_DOUBLE_EQ(dut(kS), kS + kRelativeEpsilon);
}

// Exceeding the maximum but within linear tolerance.
TEST_F(RangeValidatorRelativeEpsilonRange, ExceedsMaximum) {
  const double kS{kMax + kTolerance / 2.};
  EXPECT_DOUBLE_EQ(dut(kS), kMax - kRelativeEpsilon);
}

// Exceeding the minimum but within linear tolerance.
TEST_F(RangeValidatorRelativeEpsilonRange, ExceedsMinimum) {
  const double kS{kMin - kTolerance / 2.};
  EXPECT_DOUBLE_EQ(dut(kS), kMin + kRelativeEpsilon);
}

// Expects throw because of out of bounds.
TEST_F(RangeValidatorRelativeEpsilonRange, OutOfBounds) {
  const double kS{kMax + 10 * kTolerance};
  EXPECT_THROW({ dut(kS); }, maliput::common::assertion_error);
}

// Tests behavior when working close to the limit of the precision for both relative and absolute use of epsilon value.
// Considering that the number of useful digits for the double type is about 15(or 16) digits.
class RangeValidatorOutOfPrecisionTest : public ::testing::Test {
 protected:
  void SetUp() override {}

  static constexpr double kMin{0.5};
  static constexpr double kMax{100000.5};
  static constexpr double kTolerance{1e-3};
  static constexpr double kEpsilon{1e-14};
};

TEST_F(RangeValidatorOutOfPrecisionTest, WithAbsoluteEpsilon) {
  const RangeValidator dut{RangeValidator::GetAbsoluteEpsilonValidator(kMin, kMax, kTolerance, kEpsilon)};
  // In the maximum of the range.
  const double kS{kMax};
  // The value isn't clamped because it is beyond of the double precision.
  EXPECT_DOUBLE_EQ(dut(kS), kS);
}

TEST_F(RangeValidatorOutOfPrecisionTest, WithRelativeEpsilon) {
  const double kRelativeEpsilon{kEpsilon * (kMax - kMin)};
  const RangeValidator dut{RangeValidator::GetRelativeEpsilonValidator(kMin, kMax, kTolerance, kEpsilon)};
  // In the maximum of the range.
  const double kS{kMax};
  // The value is clamped because the epsilon value is weighten by the length of the range.
  EXPECT_DOUBLE_EQ(dut(kS), kS - kRelativeEpsilon);
}

}  // namespace test
}  // namespace common
}  // namespace maliput
