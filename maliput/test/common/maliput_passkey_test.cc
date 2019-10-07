#include "maliput/common/passkey.h"

#include <gtest/gtest.h>

namespace maliput {
namespace common {
namespace test {
namespace {

// Foo and Bar classes below and the test show how the method level friendship
// works.

class Foo;

class Bar {
 public:
  Bar() = default;
  int get_value() const { return value_; }
  void set_value(Passkey<Foo>, int value) { value_ = value; }

 private:
  int value_{};
};

class Foo {
 public:
  Foo() = default;
  void SetValueToBar(int value, Bar* bar) const { bar->set_value({}, value); }
};

GTEST_TEST(PasskeyTest, PasskeyIdiom) {
  Bar bar;
  EXPECT_EQ(bar.get_value(), 0);

  const Foo foo;
  foo.SetValueToBar(123, &bar);
  EXPECT_EQ(bar.get_value(), 123);
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
