
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "maliput/common/maliput_copyable.h"

#include <iostream>
#include <map>
#include <memory>

namespace maliput {
namespace logging {

using string_view_t = fmt::basic_string_view<char>;

namespace {

/// Logging levels.
enum level {
  trace = 0,
  debug,
  info,
  warn,
  err,
  critical,
};

/// Convenient conversion from level of message to string.
std::map<level, string_view_t> level_to_s{
    {level::trace, "[TRACE] "}, {level::debug, "[DEBUG] "}, {level::info, "[INFO] "},
    {level::warn, "[WARN] "},   {level::err, "[ERROR] "},   {level::critical, "[CRITICAL] "},
};

}  // namespace

/// Base class for the implementation of the means for the transmission of the log message.
class SinkBase {
 public:
  virtual ~SinkBase() = default;

  /// Log the message
  /// @param msg Is the message to be logged.
  virtual void log(const string_view_t& msg) = 0;

  /// Empty the buffer.
  virtual void flush() = 0;
};

/// Address the message to be displayed by stdout using a fmt format.
class Sink : public SinkBase {
 public:
  Sink() = default;
  ~Sink() = default;

  void log(const string_view_t& msg) { fmt::print(msg); }

  void flush(){};
};

/// A logger for capturing the messages and send it throw a specified mean.
///   * This logger provides six type of message's level.
///   * The sink, where the message will be delivered, must be selected using logging::set_sink() function.
///
/// To log with `maliput::logger`, you should:
///
/// <pre>
///   maliput::log()->trace("Trace message: {} {}", something, some_other);
/// </pre>
///
/// Similarly, it provides:
///
/// <pre>
///   maliput::maliput_log()->debug(...);
///   maliput::maliput_log()->info(...);
///   maliput::maliput_log()->warn(...);
///   maliput::maliput_log()->error(...);
///   maliput::maliput_log()->critical(...);
/// </pre>
class Logger {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Logger);

  Logger() = default;

  /// Log the message showing trace level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void trace(string_view_t fmt, const Args&... args) {
    log(level::trace, fmt, args...);
  }

  /// Log the message showing debug level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void debug(string_view_t fmt, const Args&... args) {
    log(level::debug, fmt, args...);
  }

  /// Log the message showing info level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void info(string_view_t fmt, const Args&... args) {
    log(level::info, fmt, args...);
  }

  /// Log the message showing warn level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void warn(string_view_t fmt, const Args&... args) {
    log(level::warn, fmt, args...);
  }

  /// Log the message showing error level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void error(string_view_t fmt, const Args&... args) {
    log(level::err, fmt, args...);
  }

  /// Log the message showing critical level prefix.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void critical(string_view_t fmt, const Args&... args) {
    log(level::critical, fmt, args...);
  }

 private:
  /// Send the message to the sink.
  /// @param lev Level of the message.
  /// @param fmt Is a format string that contains literal text and replacement fields surrounded by braces {}.
  /// @param args Is an argument list representing objects to be formatted.
  template <typename... Args>
  void log(level lev, const string_view_t& fmt, const Args&... args);
};

/// Static unique pointer that contain the current sink.
/// The logging::Sink output is selected by default.
static std::unique_ptr<logging::SinkBase> sink{std::make_unique<logging::Sink>(logging::Sink())};

/// Select a particular sink.
/// @param s Is a Sink implemented from logging::SinkBase.
void set_sink(std::unique_ptr<logging::SinkBase> s) { sink = (std::move(s)); }

template <typename... Args>
void Logger::log(level lev, const string_view_t& fmt, const Args&... args) {
  fmt::memory_buffer msg;
  format_to(msg, fmt::format(level_to_s.at(lev)));
  format_to(msg, fmt::format(fmt, args...));
  sink->log(to_string(msg));
}

}  // namespace logging

/// Retrieve an instance of a logger to use for logging.
logging::Logger* maliput_log();

}  // namespace maliput
