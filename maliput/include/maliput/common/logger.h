#pragma once

/// @file
/// To log with `maliput::logger`, you should:
///
/// <pre>
///   maliput::log()->trace("Trace message: {} {}", something, some_other);
/// </pre>
///
/// Similarly, it provides:
///
/// <pre>
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

#include <string>

/// @note spdlog support is provided by Drake's installation.

#ifdef DOXYGEN
/// A preprocessor macro set by Drake's cmake
/// configuration for spdlog at bundling time and available when
/// incorporating Drake as a dependency with it.
/// @see https://github.com/RobotLocomotion/drake/blob/master/tools/workspace/spdlog/package-create-cps.py#L28
#define HAVE_SPDLOG
#endif

#ifdef HAVE_SPDLOG

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#else  // HAVE_SPDLOG

#include "maliput/common/maliput_copyable.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

#endif  // HAVE_SPDLOG

namespace maliput {

#ifdef HAVE_SPDLOG

namespace logging {

using Logger = spdlog::logger;

using Sink = spdlog::sinks::sink;

/// True only if spdlog is enabled in this build.
constexpr bool kHaveSpdlog = true;

}  // namespace logging

#else  // HAVE_SPDLOG
namespace logging {
constexpr bool kHaveSpdlog = false;

/// A stubbed-out version of `spdlog::logger`.
/// Implements only those methods that we expect to use, as
/// spdlog's API does change from time to time.
class Logger {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Logger)

  Logger() = default;

  template <typename... Args>
  void trace(const char*, const Args&...) {}
  template <typename... Args>
  void debug(const char*, const Args&...) {}
  template <typename... Args>
  void info(const char*, const Args&...) {}
  template <typename... Args>
  void warn(const char*, const Args&...) {}
  template <typename... Args>
  void error(const char*, const Args&...) {}
  template <typename... Args>
  void critical(const char*, const Args&...) {}

  template <typename T>
  void trace(const T&) {}
  template <typename T>
  void debug(const T&) {}
  template <typename T>
  void info(const T&) {}
  template <typename T>
  void warn(const T&) {}
  template <typename T>
  void error(const T&) {}
  template <typename T>
  void critical(const T&) {}
};

// A stubbed-out version of `spdlog::sinks::sink`.
class Sink {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Sink)
  Sink() = default;
};

}  // namespace logging

#define SPDLOG_TRACE(logger, ...)
#define SPDLOG_DEBUG(logger, ...)

#endif  // HAVE_SPDLOG

/// Retrieve an instance of a logger to use for logging; for example:
///
/// <pre>
///   maliput::log()->info("potato!")
/// </pre>
///
/// See the logger.h documentation for a short tutorial.
logging::Logger* log();

namespace logging {

/// (Advanced) Retrieves the default sink for all Maliput logs.
/// When spdlog is enabled, the return value can be cast to
/// spdlog::sinks::dist_sink_mt and thus allows consumers of Maliput
/// to redirect Maliput's text logs to locations other than the default
/// of stderr.  When spdlog is disabled, the return value is an empty class.
Sink* get_dist_sink();

/// Invokes `maliput::log()->set_level(level)`.
///
/// @param level Must be a string from spdlog enumerations: `trace`, `debug`,
/// `info`, `warn`, `err`, `critical`, `off`, or `unchanged` (not an enum, but
/// useful for command-line).
///
/// @return The string value of the previous log level. When spdlog is
///         disabled, an empty string is returned.
///
/// @throws maliput::common::assertion_error When `level` is not one of the
///         predefined values for spdlog.
/// @throws maliput::common::assertion_error When log()->level() is not one
///         of the predefined values for spdlog.
std::string set_log_level(const std::string& level);

}  // namespace logging
}  // namespace maliput
