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
