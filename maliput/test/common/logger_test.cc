#include "maliput/common/logger.h"
#include "maliput/math/vector.h"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include <string>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace common {
namespace test {
namespace {

// Sink implementation that dumps the log messages in a variable.
class MockSink : public SinkBase {
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MockSink);

 public:
  MockSink() = default;
  ~MockSink() = default;

  const std::string& get_log_message() const { return log_message_; }
  void log(const std::string& msg) override { log_message_ = fmt::format(msg); }
  void flush() override{};

 private:
  std::string log_message_{};
};

GTEST_TEST(LoggerTest, GetInstance) { EXPECT_NE(log(), nullptr); }

GTEST_TEST(LoggerTest, SetSink) {
  std::unique_ptr<SinkBase> dut = std::make_unique<Sink>();
  SinkBase* dut_ptr = dut.get();
  log()->set_sink(std::move(dut));
  EXPECT_EQ(log()->get_sink(), dut_ptr);
  EXPECT_THROW(log()->set_sink(std::unique_ptr<SinkBase>()), common::assertion_error);
}

GTEST_TEST(LoggerTest, Logger) {
  std::unique_ptr<MockSink> mock_sink = std::make_unique<MockSink>();
  const MockSink* mock_sink_ptr = mock_sink.get();
  log()->set_sink(std::move(mock_sink));

  const std::string kMessage1 = " Hello World. {}{}.";
  const std::string kMessage2 = "The value of PI is: ";
  const double kPI = 3.14159;
  set_log_level(logger::kLevelToString.at(logger::level::trace));
  {
    const std::string kExpectedMessage{"[TRACE]  Hello World. The value of PI is: 3.14159.\n"};
    log()->trace(kMessage1, kMessage2, kPI);
    EXPECT_EQ(mock_sink_ptr->get_log_message(), kExpectedMessage);
  }
  {
    const std::string kExpectedMessage{"[DEBUG]  Hello World. The value of PI is: 3.14159.\n"};
    log()->debug(kMessage1, kMessage2, kPI);
    EXPECT_EQ(mock_sink_ptr->get_log_message(), kExpectedMessage);
  }
  {
    const std::string kExpectedMessage{"[INFO]  Hello World. The value of PI is: 3.14159.\n"};
    log()->info(kMessage1, kMessage2, kPI);
    EXPECT_EQ(mock_sink_ptr->get_log_message(), kExpectedMessage);
  }
  {
    const std::string kExpectedMessage{"[WARNING]  Hello World. The value of PI is: 3.14159.\n"};
    log()->warn(kMessage1, kMessage2, kPI);
    EXPECT_EQ(mock_sink_ptr->get_log_message(), kExpectedMessage);
  }
  {
    const std::string kExpectedMessage{"[ERROR]  Hello World. The value of PI is: 3.14159.\n"};
    log()->error(kMessage1, kMessage2, kPI);
    EXPECT_EQ(mock_sink_ptr->get_log_message(), kExpectedMessage);
  }
  {
    const std::string kExpectedMessage{"[CRITICAL]  Hello World. The value of PI is: 3.14159.\n"};
    log()->critical(kMessage1, kMessage2, kPI);
    EXPECT_EQ(mock_sink_ptr->get_log_message(), kExpectedMessage);
  }
}

GTEST_TEST(LoggerTest, SetLogLevel) {
  std::unique_ptr<MockSink> mock_sink = std::make_unique<MockSink>();
  MockSink* mock_sink_ptr = mock_sink.get();
  log()->set_sink(std::move(mock_sink));

  set_log_level(logger::kLevelToString.at(logger::level::trace));
  EXPECT_EQ(set_log_level(logger::kLevelToString.at(logger::level::critical)),
            logger::kLevelToString.at(logger::level::trace));

  log()->error("Hello World");
  EXPECT_EQ(mock_sink_ptr->get_log_message(), std::string{});
  EXPECT_THROW(set_log_level("wrong_level"), std::out_of_range);
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
