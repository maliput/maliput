#include "maliput/common/maliput_abort.h"

#include <gtest/gtest.h>

namespace maliput {
namespace common {
namespace test {
namespace {

// Evaluates whether or not MALIPUT_DEMAND() aborts.
GTEST_TEST(MaliputDemandDeathTest, DemandTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { MALIPUT_DEMAND(false); },
      "abort: Failure at .*maliput_abort_test.cc:.. in TestBody..: "
      "condition 'false' failed");
}

// Evaluates that MALIPUT_ABORT_MSG() aborts.
GTEST_TEST(MaliputAbortDeathTest, AbortMsgTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { MALIPUT_ABORT_MESSAGE("I'm gonna die!"); },
      "abort: Failure at .*maliput_abort_test.cc:.. in TestBody..: "
      "condition '' failed. Details: \"I'm gonna die!\"");
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
