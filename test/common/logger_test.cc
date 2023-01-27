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
#include "maliput/common/logger.h"

#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/math/vector.h"

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
  void log(const std::string& msg) override { log_message_ = msg; }
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

class LoggingMesagesTest : public ::testing::Test {
 protected:
  static constexpr char const* kMessage1 = " Hello World. {}{}.";
  static constexpr char const* kMessage2 = "The value of PI is: ";
  static constexpr char const* kExpectedMessage = "  Hello World. The value of PI is: 3.14159.\n";
  static constexpr double kPI = 3.14159;

  void SetUp() override {
    mock_sink_ptr = mock_sink.get();
    log()->set_sink(std::move(mock_sink));
    set_log_level(logger::kLevelToString.at(logger::level::trace));
  }

  std::unique_ptr<MockSink> mock_sink = std::make_unique<MockSink>();
  MockSink* mock_sink_ptr{nullptr};
};

TEST_F(LoggingMesagesTest, EscapingFormatting) {
  // Double curly braces escapes fmt formating.
  const std::string message{"Escape {{formatting}}"};
  const std::string expected_message{"[TRACE] Escape {formatting}\n"};
  log()->trace(message);
  EXPECT_EQ(mock_sink_ptr->get_log_message(), expected_message);
}

TEST_F(LoggingMesagesTest, TraceLog) {
  const std::string expected_message{"[TRACE]" + std::string(kExpectedMessage)};
  log()->trace(kMessage1, kMessage2, kPI);
  EXPECT_EQ(mock_sink_ptr->get_log_message(), expected_message);
}

TEST_F(LoggingMesagesTest, DebugLog) {
  const std::string expected_message{"[DEBUG]" + std::string(kExpectedMessage)};
  log()->debug(kMessage1, kMessage2, kPI);
  EXPECT_EQ(mock_sink_ptr->get_log_message(), expected_message);
}

TEST_F(LoggingMesagesTest, InfoLog) {
  const std::string expected_message{"[INFO]" + std::string(kExpectedMessage)};
  log()->info(kMessage1, kMessage2, kPI);
  EXPECT_EQ(mock_sink_ptr->get_log_message(), expected_message);
}

TEST_F(LoggingMesagesTest, WarningLog) {
  const std::string expected_message{"[WARNING]" + std::string(kExpectedMessage)};
  log()->warn(kMessage1, kMessage2, kPI);
  EXPECT_EQ(mock_sink_ptr->get_log_message(), expected_message);
}

TEST_F(LoggingMesagesTest, ErrorLog) {
  const std::string expected_message{"[ERROR]" + std::string(kExpectedMessage)};
  log()->error(kMessage1, kMessage2, kPI);
  EXPECT_EQ(mock_sink_ptr->get_log_message(), expected_message);
}

TEST_F(LoggingMesagesTest, CriticalLog) {
  const std::string expected_message{"[CRITICAL]" + std::string(kExpectedMessage)};
  log()->critical(kMessage1, kMessage2, kPI);
  EXPECT_EQ(mock_sink_ptr->get_log_message(), expected_message);
}

TEST_F(LoggingMesagesTest, OffLevelLog) {
  log()->set_level(logger::level::off);
  const std::string expected_empty_message{};
  log()->critical(kMessage1, kMessage2, kPI);
  EXPECT_EQ(mock_sink_ptr->get_log_message(), expected_empty_message);
}

}  // namespace
}  // namespace test
}  // namespace common
}  // namespace maliput
