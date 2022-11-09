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
#include "maliput/common/maliput_throw.h"

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace common {
namespace test {
namespace {

// Evaluates whether or not MALIPUT_THROW_UNLESS() throws.
GTEST_TEST(MaliputThrowTest, ExpectThrowAndNoThrowTest) {
  EXPECT_THROW({ MALIPUT_THROW_UNLESS(false); }, assertion_error);
  EXPECT_NO_THROW({ MALIPUT_THROW_UNLESS(true); });
}

// Evaluates whether or not MALIPUT_THROW_UNLESS() throws.
GTEST_TEST(MaliputThrowMessageTest, ExpectThrowWithMessageTest) {
  EXPECT_THROW({ MALIPUT_THROW_MESSAGE("Exception description"); }, assertion_error);
}

// Evaluates whether or not MALIPUT_VALIDATE() throws.
GTEST_TEST(MaliputValidateTest, Test) {
  EXPECT_THROW({ MALIPUT_VALIDATE(false, "Exception description"); }, assertion_error);
  EXPECT_NO_THROW({ MALIPUT_VALIDATE(true, "Exception description"); });
}

// Evaluates whether or not MALIPUT_IS_IN_RANGE() throws.
GTEST_TEST(MaliputIsInRangeTest, Test) {
  EXPECT_THROW({ MALIPUT_IS_IN_RANGE(5., 1., 4.); }, assertion_error);
  EXPECT_NO_THROW({ MALIPUT_IS_IN_RANGE(5., 1., 10.); });
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
