#include "maliput/common/maliput_logger.h"

#include <string>

#include <gtest/gtest.h>

namespace maliput {
namespace common {
namespace test {
namespace {

GTEST_TEST(LoggerTest, Logger) {
  const std::string f = "world";
  maliput_log()->info("hello {}, {},\n", f, 55);
  maliput_log()->error("hello {}, {},\n", f, 55);
  maliput_log()->debug("hello {}, {},\n", f, 55);
  maliput_log()->critical("hello {}, {}\n", f, 55);
  maliput_log()->warn("hello {}, {}\n", f, 55);
  maliput_log()->trace("hello {}, {}\n", f, 55);
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
