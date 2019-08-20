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

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
