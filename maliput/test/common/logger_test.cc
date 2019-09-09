#include "maliput/common/logger.h"

#include <string>

#include <gtest/gtest.h>

#ifdef HAVE_SPDLOG
#include <spdlog/sinks/dist_sink.h>
#endif  // HAVE_SPDLOG

namespace maliput {
namespace common {
namespace test {
namespace {

// Evaluates the flag pointing to the internal use of spdlog when available.
GTEST_TEST(LoggerTest, HaveSpdlogFlagTest) {
#ifdef HAVE_SPDLOG
  ASSERT_TRUE(logging::kHaveSpdlog);
#else   // HAVE_SPDLOG
  ASSERT_FALSE(logging::kHaveSpdlog);
#endif  // HAVE_SPDLOG
}

// Regardless of having spdlog or not, the Logger pointer must not be nullptr.
GTEST_TEST(LoggerTest, NonNullptrLogger) { ASSERT_NE(log(), nullptr); }

// Abuse gtest internals to verify that logging actually prints when enabled,
// and that the default level is INFO.
GTEST_TEST(LoggerTest, CaptureOutputTest) {
  testing::internal::CaptureStderr();
  log()->debug("sample_debug_message");
  log()->info("sample_info_message");
  const std::string output_log = testing::internal::GetCapturedStderr();
#ifdef HAVE_SPDLOG
  EXPECT_TRUE(output_log.find("sample_info_message") != std::string::npos);
  EXPECT_TRUE(output_log.find("sample_debug_message") == std::string::npos);
#else   // HAVE_SPDLOG
  EXPECT_EQ(output_log, "");
#endif  // HAVE_SPDLOG
}

// Regardless of having spdlog or not, the Sink pointer must not be nullptr.
GTEST_TEST(LoggerTest, NonNullptrSink) { ASSERT_NE(logging::get_dist_sink(), nullptr); }

// When having spdlog, evaluates that the Sink can be casted to the subtype in the docstring.
#ifdef HAVE_SPDLOG
GTEST_TEST(LoggerTest, SinkTest) {
  ASSERT_NE(dynamic_cast<spdlog::sinks::dist_sink_mt*>(logging::get_dist_sink()), nullptr);
}
#endif

GTEST_TEST(LoggerTest, SetLogLevel) {
#ifdef HAVE_SPDLOG
  const std::vector<std::string> kLogLevels{"trace", "debug", "info", "warn", "err", "critical", "off"};
  const std::string first_level = logging::set_log_level("unchanged");
  std::string prev_level = "off";
  logging::set_log_level(prev_level);
  for (const std::string& level : kLogLevels) {
    EXPECT_EQ(logging::set_log_level(level), prev_level);
    prev_level = level;
  }
  logging::set_log_level(first_level);
#else   // HAVE_SPDLOG
  ASSERT_EQ(logging::set_log_level("any level name"), "");
#endif  // HAVE_SPDLOG
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
