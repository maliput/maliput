#include "maliput/common/maliput_deprecated.h"

#include <gtest/gtest.h>

#include "maliput/common/maliput_unused.h"

// This test verifies that the maliput build still succeeds if a deprecated class
// or function is in use.

namespace maliput {
namespace common {
namespace {

class MALIPUT_DEPRECATED("2027-05-27", "Use MyNewClass instead.") MyOldClass {};

class MyNewClass {};

MALIPUT_DEPRECATED("2038-01-19", "Don't use this function; use NewMethod() instead.")
int OldMethod(int arg) { return arg; }

int NewMethod(int arg) { return arg; }

GTEST_TEST(MaliputDeprecatedTest, ClassTest) {
  const MyOldClass deprecated;
  const MyNewClass not_deprecated;
  unused(deprecated, not_deprecated);
}

GTEST_TEST(MaliputDeprecatedTest, FunctionTest) {
  const int deprecated = OldMethod(1);
  const int not_deprecated = NewMethod(1);
  unused(deprecated, not_deprecated);
}

}  // namespace
}  // namespace common
}  // namespace maliput
