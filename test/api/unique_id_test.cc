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
#include "maliput/api/unique_id.h"

#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace api {
namespace {

GTEST_TEST(UniqueIdTest, Construction) {
  EXPECT_NO_THROW(UniqueId("x"));
  EXPECT_THROW(UniqueId(""), maliput::common::assertion_error);
}

GTEST_TEST(UniqueIdTest, Accessors) {
  const UniqueId dut("x");
  EXPECT_EQ(dut.string(), "x");
}

GTEST_TEST(UniqueIdTest, Equality) {
  const UniqueId dut_x1("x");
  const UniqueId dut_x2("x");
  const UniqueId dut_y("y");

  EXPECT_TRUE(dut_x1 == dut_x2);
  EXPECT_FALSE(dut_x1 != dut_x2);

  EXPECT_FALSE(dut_x1 == dut_y);
  EXPECT_TRUE(dut_x1 != dut_y);
}

GTEST_TEST(UniqueIdTest, CopyingAndAssignment) {
  const UniqueId dut1("x");
  const UniqueId dut2(dut1);
  UniqueId dut3("y");
  dut3 = dut1;

  EXPECT_TRUE(dut1 == dut2);
  EXPECT_TRUE(dut1 == dut3);
}

// Test usage with ordered/unordered sets.
template <typename T>
class UniqueIdSetTest : public ::testing::Test {};

typedef ::testing::Types<std::set<UniqueId>, std::unordered_set<UniqueId>> SetTypes;

TYPED_TEST_CASE(UniqueIdSetTest, SetTypes);

TYPED_TEST(UniqueIdSetTest, SetTypes) {
  TypeParam dut;

  dut.insert(UniqueId("a"));
  dut.insert(UniqueId("b"));
  dut.insert(UniqueId("c"));
  // Insert a fresh, duplicate instance of "a".
  dut.insert(UniqueId("a"));

  EXPECT_EQ(static_cast<int>(dut.size()), 3);
  EXPECT_EQ(static_cast<int>(dut.count(UniqueId("a"))), 1);
  EXPECT_EQ(static_cast<int>(dut.count(UniqueId("b"))), 1);
  EXPECT_EQ(static_cast<int>(dut.count(UniqueId("c"))), 1);
}

// Test usage with ordered/unordered maps.
template <typename T>
class UniqueIdMapTest : public ::testing::Test {};

typedef ::testing::Types<std::map<UniqueId, int>, std::unordered_map<UniqueId, int>> MapTypes;

TYPED_TEST_CASE(UniqueIdMapTest, MapTypes);

TYPED_TEST(UniqueIdMapTest, MapTypes) {
  TypeParam dut;

  dut[UniqueId("a")] = 1;
  dut[UniqueId("b")] = 2;
  dut[UniqueId("c")] = 3;
  // Insert a fresh, duplicate instance of "a".
  dut[UniqueId("a")] = 5;

  EXPECT_EQ(static_cast<int>(dut.size()), 3);
  EXPECT_EQ(dut[UniqueId("a")], 5);
  EXPECT_EQ(dut[UniqueId("b")], 2);
  EXPECT_EQ(dut[UniqueId("c")], 3);
}

}  // namespace
}  // namespace api
}  // namespace maliput
