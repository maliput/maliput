#pragma once

/// @file
/// To log with `maliput::common::logger`, you should:
///
/// <pre>
///   maliput::log()->trace("The approximate value is ", 3.14159);
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
/// Any class that overloads `operator<<` for `ostream` can be
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

#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace common {

namespace logger {

/// Logging levels.
enum level {
  trace = 0,
  debug,
  info,
  warn,
  error,
  critical,
  off,
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

/// Convenient conversion from level enum to message.
const std::map<int, std::string> kLevelToMessage{
    {logger::level::trace, "[TRACE] "},  {logger::level::debug, "[DEBUG] "}, {logger::level::info, "[INFO] "},
    {logger::level::warn, "[WARNING] "}, {logger::level::error, "[ERROR] "}, {logger::level::critical, "[CRITICAL] "},
};

}  // namespace logger

/// Interface of a sink to dump all log messages.
class SinkBase {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(SinkBase);
  SinkBase() = default;
  virtual ~SinkBase() = default;

  /// Log the message
  /// @param msg Is the message to be logged.
  virtual void log(const std::string& msg) = 0;

  /// Empty the buffer.
  virtual void flush() = 0;
};

/// Sink that uses `std::cout` to dump the log messages.
class Sink : public SinkBase {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Sink);
  Sink() = default;
  ~Sink() = default;

  void log(const std::string& msg) override;

  void flush() override{};
};

/// A logger class implementation.
///
/// Logger will dump all messages to a sink (@ref SinkBase) which will be in charge
/// of serializing the messages to the appropriate channel. By default, Sink
/// implementation is used.
///
/// It provides six different log levels, @ref logger::level , which can be filtered based
/// on the severity of the message.
///
/// Comments about the design:
/// - Within Logger::log() method the variadic arguments are unpacked and serialized using a functor @ref Serialize .
///   The alternative to the functor is to use a lambda expresion, but there is a bug in gcc that is not fixed until 8.1
///   version.
///   @see GCC bug: https://gcc.gnu.org/bugzilla/show_bug.cgi?id=85305.
/// - Another way to do type-erasure is using std::any to allocate the variadic arguments, but this idea
///   was dismissed given that it wouldn't allow the logger to manage all kind of types, due to the fact that
///   an overload of any_cast<T>() for each new type to be serialized would be required.
class Logger {
 public:
  /// Indicates the maximum number of arguments that a single log command is able to process.
  static constexpr int kNumberOfArguments{100};

  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Logger);

  Logger() = default;

  /// \addtogroup levelmethods Logging Level Methods.
  /// @param args Is an argument list representing objects to be formatted.
  /// @{

  /// Log the message showing trace level prefix.
  template <typename... Args>
  void trace(const Args&... args) {
    log(logger::level::trace, args...);
  }

  /// Log the message showing debug level prefix.
  template <typename... Args>
  void debug(const Args&... args) {
    log(logger::level::debug, args...);
  }

  /// Log the message showing info level prefix.
  template <typename... Args>
  void info(const Args&... args) {
    log(logger::level::info, args...);
  }

  /// Log the message showing warning level prefix.
  template <typename... Args>
  void warn(const Args&... args) {
    log(logger::level::warn, args...);
  }

  /// Log the message showing error level prefix.
  template <typename... Args>
  void error(const Args&... args) {
    log(logger::level::error, args...);
  }

  /// Log the message showing critical level prefix.
  template <typename... Args>
  void critical(const Args&... args) {
    log(logger::level::critical, args...);
  }
  /// @}

  /// Set a sink.
  /// @param sink Is a SinkBase implementation.
  ///
  /// @throw common::assertion_error When `sink` is nullptr.
  void set_sink(std::unique_ptr<common::SinkBase> sink);

  /// Get the current sink.
  /// @return A pointer to the current sink.
  SinkBase* get_sink() { return sink_.get(); }

  /// Sets the minimum level of messages to be logged.
  /// @param log_level Must be a level enum value from the level enumerations: `trace`, `debug`,
  /// `info`, `warning`, `error`, `critical` or `off`.
  /// @return The string value of the previous log level. @see logger::kLevelToString.
  std::string set_level(logger::level log_level);

 private:
  // Send the message to the sink.
  // @param log_level Level of the message.
  // @param args Is an argument list to be processed as a string.
  template <typename... Args>
  void log(logger::level log_level, Args&&... args);

  // Sink where the messages will be dumped to.
  std::unique_ptr<common::SinkBase> sink_{std::make_unique<common::Sink>()};

  // Minimum level of messages to be log.
  logger::level level_{logger::level::info};

  // Create a string from a list of arguments.
  // @tparam Args Variadic arguments to generate the list of arguments.
  // @param args Is an argument list representing objects with an already defined serialization applied.
  // @return A string obtained by concatenating the arguments.
  template <typename... Args>
  const std::string format(Args&&... args) const {
    std::string result;
    ((result += args), ...);
    return result;
  }
};

/// Convenient functor for getting a string from a object that has serialization operator defined.
struct Serialize {
  /// @param t Object to apply the serialization.
  /// @tparam T Type of t.
  /// @return A string obtained from `t`
  /// @see Logger.
  template <typename T>
  std::string operator()(const T& t) {
    std::stringstream os;
    os << t;
    return os.str();
  }
};

template <typename... Args>
void Logger::log(logger::level lev, Args&&... args) {
  if (lev >= level_) {
    std::string msg{};
    msg += logger::kLevelToMessage.at(lev);
    msg += format(Serialize()(std::forward<Args>(args))...);
    msg += "\n";
    sink_->log(msg);
  }
}

/// Invokes `maliput::log()->set_level(level)`.
/// See @relates Logger Logger.
///
/// @param level Must be a string from the level enumerations: `trace`, `debug`,
///              `info`, `warning`, `error`, `critical` or `off`.
/// @return The string value of the previous log level.
///
/// @throw std::out_of_range When `level` is not one of the
///         predefined values.
std::string set_log_level(const std::string& level);

}  // namespace common

/// Retrieve an instance of a logger to use for logger.
/// See @relates common::Logger Logger.
common::Logger* log();

}  // namespace maliput
