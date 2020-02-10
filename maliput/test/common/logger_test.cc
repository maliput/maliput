#include "maliput/common/logger.h"

#include <string>
#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace common {
namespace test {
namespace {

// Contains the value of a log line.
struct SinkMsg {
  std::string msg{};
} result;

// Sink implementation that dumps the log messages in a variable.
class MockSink : public SinkBase {
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockSink);

 public:
  MockSink() = default;
  ~MockSink() = default;

  void log(const string_view_t& msg) override { result.msg = fmt::format(msg); }

  void flush() override{};
};

GTEST_TEST(LoggerTest, GetInstance) { EXPECT_EQ(log(), log()); }

GTEST_TEST(LoggerTest, SetSink) {
  std::unique_ptr<SinkBase> dut_1 = std::make_unique<Sink>();
  log()->set_sink(std::move(dut_1));
  std::unique_ptr<SinkBase> dut_2 = std::make_unique<Sink>();
  log()->set_sink(std::move(dut_2));
  EXPECT_THROW(log()->set_sink(std::move(dut_1)), common::assertion_error);
}

GTEST_TEST(LoggerTest, Logger) {
  std::unique_ptr<MockSink> mock_sink = std::make_unique<MockSink>();
  log()->set_sink(std::move(mock_sink));

  const std::string kString = "The value of PI is: ";
  const double kPI = 3.1415926535;
  set_log_level(logger::kLevelToString.at(logger::level::trace));
  {
    const std::string kMsg{"[TRACE]  Hello World. The value of PI is: 3.1415926535.\n"};
    log()->trace(" Hello World. {}{}.\n", kString, kPI);
    EXPECT_EQ(result.msg, kMsg);
  }
  {
    const std::string kMsg{"[DEBUG]  Hello World. The value of PI is: 3.1415926535.\n"};
    log()->debug(" Hello World. {}{}.\n", kString, kPI);
    EXPECT_EQ(result.msg, kMsg);
  }
  {
    const std::string kMsg{"[INFO]  Hello World. The value of PI is: 3.1415926535.\n"};
    log()->info(" Hello World. {}{}.\n", kString, kPI);
    EXPECT_EQ(result.msg, kMsg);
  }
  {
    const std::string kMsg{"[WARNING]  Hello World. The value of PI is: 3.1415926535.\n"};
    log()->warn(" Hello World. {}{}.\n", kString, kPI);
    EXPECT_EQ(result.msg, kMsg);
  }
  {
    const std::string kMsg{"[ERROR]  Hello World. The value of PI is: 3.1415926535.\n"};
    log()->error(" Hello World. {}{}.\n", kString, kPI);
    EXPECT_EQ(result.msg, kMsg);
  }
  {
    const std::string kMsg{"[CRITICAL]  Hello World. The value of PI is: 3.1415926535.\n"};
    log()->critical(" Hello World. {}{}.\n", kString, kPI);
    EXPECT_EQ(result.msg, kMsg);
  }
}

GTEST_TEST(LoggerTest, SetLogLevel) {
  result.msg = "";
  EXPECT_EQ(set_log_level(logger::kLevelToString.at(logger::level::critical)),
            logger::kLevelToString.at(logger::level::trace));
  {
    log()->error("Hello World");
    EXPECT_EQ(result.msg, std::string{});
  }
  EXPECT_THROW(set_log_level("wrong_level"), common::assertion_error);
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
