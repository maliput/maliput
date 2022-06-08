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
#include "maliput/api/type_specific_identifier.h"

#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace api {
namespace {

class C {};  // some random class

using CId = TypeSpecificIdentifier<C>;  // ID type specific to class C

GTEST_TEST(TypeSpecificIdentifierTest, Construction) {
  EXPECT_NO_THROW(CId("x"));
  EXPECT_THROW(CId(""), maliput::common::assertion_error);
}

// clang-format off
GTEST_TEST(TypeSpecificIdentifierTest, IdentifiedType) {
  ::testing::StaticAssertTypeEq<CId::identified_type, C>();
}
// clang-format on

GTEST_TEST(TypeSpecificIdentifierTest, Accessors) {
  const CId dut("x");
  EXPECT_EQ(dut.string(), "x");
}

GTEST_TEST(TypeSpecificIdentifierTest, Equality) {
  const CId dut_x1("x");
  const CId dut_x2("x");
  const CId dut_y("y");

  EXPECT_TRUE(dut_x1 == dut_x2);
  EXPECT_FALSE(dut_x1 != dut_x2);

  EXPECT_FALSE(dut_x1 == dut_y);
  EXPECT_TRUE(dut_x1 != dut_y);
}

GTEST_TEST(TypeSpecificIdentifierTest, CopyingAndAssignment) {
  const CId dut1("x");
  const CId dut2(dut1);
  CId dut3("y");
  dut3 = dut1;

  EXPECT_TRUE(dut1 == dut2);
  EXPECT_TRUE(dut1 == dut3);
}

GTEST_TEST(TypeSpecificIdentifierTest, StreamOperator) {
  const CId dut("x");
  std::stringstream ss;
  ss << dut;
  const std::string dut_str = ss.str();
  EXPECT_EQ(dut_str, dut.string());
}

// Test usage with ordered/unordered sets.
template <typename T>
class TypeSpecificIdentifierSetTest : public ::testing::Test {};

typedef ::testing::Types<std::set<CId>, std::unordered_set<CId>> SetTypes;

TYPED_TEST_CASE(TypeSpecificIdentifierSetTest, SetTypes);

TYPED_TEST(TypeSpecificIdentifierSetTest, SetTypes) {
  TypeParam dut;

  dut.insert(CId("a"));
  dut.insert(CId("b"));
  dut.insert(CId("c"));
  // Insert a fresh, duplicate instance of "a".
  dut.insert(CId("a"));

  EXPECT_EQ(static_cast<int>(dut.size()), 3);
  EXPECT_EQ(static_cast<int>(dut.count(CId("a"))), 1);
  EXPECT_EQ(static_cast<int>(dut.count(CId("b"))), 1);
  EXPECT_EQ(static_cast<int>(dut.count(CId("c"))), 1);
}

// Test usage with ordered/unordered maps.
template <typename T>
class TypeSpecificIdentifierMapTest : public ::testing::Test {};

typedef ::testing::Types<std::map<CId, int>, std::unordered_map<CId, int>> MapTypes;

TYPED_TEST_CASE(TypeSpecificIdentifierMapTest, MapTypes);

TYPED_TEST(TypeSpecificIdentifierMapTest, MapTypes) {
  TypeParam dut;

  dut[CId("a")] = 1;
  dut[CId("b")] = 2;
  dut[CId("c")] = 3;
  // Insert a fresh, duplicate instance of "a".
  dut[CId("a")] = 5;

  EXPECT_EQ(static_cast<int>(dut.size()), 3);
  EXPECT_EQ(dut[CId("a")], 5);
  EXPECT_EQ(dut[CId("b")], 2);
  EXPECT_EQ(dut[CId("c")], 3);
}

}  // namespace
}  // namespace api
}  // namespace maliput
