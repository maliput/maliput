#pragma once

/// @file
/// To log with `maliput::common::logger`, you should:
///
/// <pre>
///   maliput::log()->trace("Trace message: {} {}", something, some_other);
/// </pre>
///
/// Similarly, it provides:
///
/// <pre>
///   maliput::log()->trace(...);
///   maliput::log()->debug(...);
///   maliput::log()->info(...);
///   maliput::log()->warn(...);
///   maliput::log()->error(...);
///   maliput::log()->critical(...);
/// </pre>
///
///
/// The format string syntax is fmtlib; see http://fmtlib.net/5.3.0/syntax.html.
/// In particular, any class that overloads `operator<<` for `ostream` can be
/// printed without any special handling.

/// Code in this file is inspired by:
/// https://github.com/RobotLocomotion/drake/blob/master/common/text_logging.h
///
/// Drake's license follows:
///
/// All components of Drake are licensed under the BSD 3-Clause License
/// shown below. Where noted in the source code, some portions may
/// be subject to other permissive, non-viral licenses.
///
/// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are
/// met:
///
/// Redistributions of source code must retain the above copyright notice,
/// this list of conditions and the following disclaimer.  Redistributions
/// in binary form must reproduce the above copyright notice, this list of
/// conditions and the following disclaimer in the documentation and/or
/// other materials provided with the distribution.  Neither the name of
/// the Massachusetts Institute of Technology nor the names of its
/// contributors may be used to endorse or promote products derived from
/// this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
/// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
/// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
/// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
/// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
/// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
/// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
/// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace common {

using string_view_t = fmt::basic_string_view<char>;

namespace logger {

/// Logging levels.
enum level {
  off = 0,
  trace,
  debug,
  info,
  warn,
  error,
  critical,
  unchanged,
};

/// Convenient conversion from string to level enum.
const std::map<std::string, logger::level> kStringToLevel{
    {std::string{"off"}, logger::level::off},           {std::string{"trace"}, logger::level::trace},
    {std::string{"debug"}, logger::level::debug},       {std::string{"info"}, logger::level::info},
    {std::string{"warn"}, logger::level::warn},         {std::string{"error"}, logger::level::error},
    {std::string{"critical"}, logger::level::critical}, {std::string{"unchanged"}, logger::level::unchanged}};

/// Convenient conversion from level enum to string.
const std::map<int, std::string> kLevelToString{
    {logger::level::off, std::string{"off"}},           {logger::level::trace, std::string{"trace"}},
    {logger::level::debug, std::string{"debug"}},       {logger::level::info, std::string{"info"}},
    {logger::level::warn, std::string{"warn"}},         {logger::level::error, std::string{"error"}},
    {logger::level::critical, std::string{"critical"}}, {logger::level::unchanged, std::string{"unchanged"}}};

}  // namespace logger

namespace {

/// Convenient conversion from level enum to message.
const std::map<int, std::string> kLevelToMessage{
    {logger::level::trace, "[TRACE] "},  {logger::level::debug, "[DEBUG] "}, {logger::level::info, "[INFO] "},
    {logger::level::warn, "[WARNING] "}, {logger::level::error, "[ERROR] "}, {logger::level::critical, "[CRITICAL] "},
};

}  // namespace

/// Interface of a sink to dump all log messages.
class SinkBase {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(SinkBase);
  SinkBase() = default;
  virtual ~SinkBase() = default;

  /// Log the message
  /// @param msg Is the message to be logged.
  virtual void log(const string_view_t& msg) = 0;

  /// Empty the buffer.
  virtual void flush() = 0;
};

/// Sink that uses `std::cout` to dump the log messages.
class Sink : public SinkBase {
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Sink);

 public:
  Sink() = default;
  ~Sink() = default;

  void log(const string_view_t& msg) override { fmt::print(msg); }

  void flush() override{};
};

/// A logger for capturing the messages and dumping them in a specified sink.
///   * The logger provides six type of message's level(trace, debug, info, warning, error, critical).
///   * The logger could be "turned off" using maliput::log()->set_log_level(logging::level::off).
///   * The sink, where the message will be delivered, must be selected using the set_sink() method.
class Logger {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Logger);

  Logger() = default;

  /// Log the message showing trace level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void trace(string_view_t fmt, const Args&... args) {
    log(logger::level::trace, fmt, args...);
  }

  /// Log the message showing debug level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void debug(string_view_t fmt, const Args&... args) {
    log(logger::level::debug, fmt, args...);
  }

  /// Log the message showing info level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void info(string_view_t fmt, const Args&... args) {
    log(logger::level::info, fmt, args...);
  }

  /// Log the message showing warning level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void warn(string_view_t fmt, const Args&... args) {
    log(logger::level::warn, fmt, args...);
  }

  /// Log the message showing error level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void error(string_view_t fmt, const Args&... args) {
    log(logger::level::error, fmt, args...);
  }

  /// Log the message showing critical level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void critical(string_view_t fmt, const Args&... args) {
    log(logger::level::critical, fmt, args...);
  }

  /// Select a particular sink.
  /// @param s Is a Sink implemented from common::SinkBase.
  ///
  /// @throw common::assertion_error When `s` is nullptr.
  void set_sink(std::unique_ptr<common::SinkBase> s);

  /// Get the current sink.
  /// @return A pointer to the current sink.
  SinkBase* get_sink() { return sink_.get(); }

  /// Sets the minimum level of messages to be log.
  /// @param level Must be a level enum value from the level enumerations: `trace`, `debug`,
  /// `info`, `warning`, `error`, `critical` or `off`.
  /// @return The string value of the previous log level.
  std::string set_level(logger::level l);

 private:
  /// Send the message to the sink.
  /// @param lev Level of the message.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void log(logger::level lev, const string_view_t& fmt, const Args&... args);

  /// Sink where the messages will be dumped to.
  std::unique_ptr<common::SinkBase> sink_{std::make_unique<common::Sink>()};

  /// Minimum level of messages to be log.
  std::atomic<int> level_{logger::level::info};
};

template <typename... Args>
void Logger::log(logger::level lev, const string_view_t& fmt, const Args&... args) {
  if (lev >= level_) {
    fmt::memory_buffer msg;
    format_to(msg, fmt::format(kLevelToMessage.at(lev)));
    format_to(msg, fmt::format(fmt, args...));
    sink_->log(to_string(msg));
  }
}

/// Invokes `maliput::log()->set_level(level)`.
///
/// @param level Must be a string from the level enumerations: `trace`, `debug`,
/// `info`, `warning`, `error`, `critical` or `off`.
/// @return The string value of the previous log level.
///
/// @throws maliput::common::assertion_error When `level` is not one of the
///         predefined values.
std::string set_log_level(const std::string& level);

}  // namespace common

/// Retrieve an instance of a logger to use for logger.
common::Logger* log();

}  // namespace maliput
