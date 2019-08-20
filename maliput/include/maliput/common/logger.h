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
/// https://github.com/RobotLocomotion/drake/blob/master/common/text_logging.hs
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

#include "drake/common/text_logging.h"

namespace maliput {

using Logger = drake::logging::logger;

/// Retrieve an instance of a logger to use for logging; for example:
///
/// <pre>
///   maliput::log()->info("potato!")
/// </pre>
///
/// See the logger.h documentation for a short tutorial.
Logger* log();

/// Invokes `maliput::log()->set_level(level)`.
///
/// @param level Must be a string from spdlog enumerations: `trace`, `debug`,
/// `info`, `warn`, `err`, `critical`, `off`, or `unchanged` (not an enum, but
/// useful for command-line).
///
/// @return The string value of the previous log level. If SPDLOG is disabled,
/// then this returns an empty string.
std::string set_log_level(const std::string& level);

}  // namespace maliput
