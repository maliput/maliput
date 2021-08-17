/// @file logger.cc
///
/// Code in this file is inspired by:
/// https://github.com/RobotLocomotion/drake/blob/master/common/text_logging.cc
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

#include "maliput/common/logger.h"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/printf.h>

#include "maliput/common/maliput_never_destroyed.h"

// fmt major version for focal: 6.
// fmt major version for bionic: 4.
#if MALIPUT_FMT_VERSION == 6
#include <fmt/color.h>
#endif

namespace maliput {
namespace common {

namespace {

#if MALIPUT_FMT_VERSION == 4
fmt::Color FromLogLevelToColor(const logger::level& lev) {
  switch (lev) {
    case logger::level::trace:
      return fmt::Color::WHITE;
      break;

    case logger::level::debug:
      return fmt::Color::BLUE;
      break;

    case logger::level::info:
      return fmt::Color::GREEN;
      break;

    case logger::level::error:
      return fmt::Color::RED;
      break;

    case logger::level::warn:
      return fmt::Color::YELLOW;
      break;

    case logger::level::critical:
      return fmt::Color::RED;
      break;

    default:
      return fmt::Color::WHITE;
  }
}
#elif MALIPUT_FMT_VERSION == 6
fmt::color FromLogLevelToColor(const logger::level& lev) {
  switch (lev) {
    case logger::level::trace:
      return fmt::color::white;
      break;

    case logger::level::debug:
      return fmt::color::blue;
      break;

    case logger::level::info:
      return fmt::color::green;
      break;

    case logger::level::error:
      return fmt::color::red;
      break;

    case logger::level::warn:
      return fmt::color::yellow;
      break;

    case logger::level::critical:
      return fmt::color::red;
      break;

    default:
      return fmt::color::white;
  }
}
#endif

// Apply fmt::format to a list of arguments defined by an array `v`.
// @tparam N Is the size of the array.
// @tparam T Is the type of the variable that contains the array.
// @tparam args Variadic arguments to generate the list of arguments.
// @param array Is the array that contains the values to be converted in arguments.
// @param index_sequence Is the index sequence to indicate the index of the array.
template <std::size_t N, typename T, std::size_t... args>
auto call_fmt_format_helper(const std::array<T, N>& array, std::index_sequence<args...> index_sequence) {
  return fmt::format(std::get<args>(array)...);
}

// Wrapper for call_fmt_format_helper function.
// Creates an index sequence from an array and pass that as an argument to call_fmt_format_helper function.
// Intermediate function to obtain a list of arguments from an array.
// @tparam N Is the size of the array.
// @tparam T Is the type of the variable that contains the array.
// @param array Is the array that contains the values to be converted in arguments.
// @see https://stackoverflow.com/questions/41467721/conversion-of-vector-or-array-to-a-list-of-arguments-in-c
template <std::size_t N, typename T>
auto call_fmt_format(const std::array<T, N>& array) {
  return call_fmt_format_helper<N>(array, std::make_index_sequence<N>());
}

}  // namespace

std::string Logger::set_level(logger::level log_level) {
  if (log_level == logger::level::unchanged) {
    return logger::kLevelToString.at(level_);
  } else {
    const logger::level previous_log_level = level_;
    level_ = log_level;
    return logger::kLevelToString.at(previous_log_level);
  }
}

bool Logger::set_color(bool color) {
  const bool ret{color_};
  if (color_ != color) {
    set_sink(std::make_unique<common::Sink>(color));
    color_ = color;
  }
  return ret;
}

void Logger::set_sink(std::unique_ptr<common::SinkBase> sink) {
  MALIPUT_THROW_UNLESS(sink.get() != nullptr);
  sink_ = (std::move(sink));
}

std::string set_log_level(const std::string& level) {
  return log()->set_level(common::logger::kStringToLevel.at(level));
}

bool set_log_color(bool color) { return log()->set_color(color); }

const std::string Logger::format(const std::vector<std::string>& v) const {
  std::array<std::string, kNumberOfArguments> args;
  for (int i = 0; i < static_cast<int>(v.size()) && i < kNumberOfArguments; i++) {
    args[i] = v[i];
  }
  return call_fmt_format(args);
}

Sink::Sink(bool color) {
  if (color) {
    print_ = [](const std::string& msg, logger::level lev) {
#if MALIPUT_FMT_VERSION == 6
      fmt::print(fg(FromLogLevelToColor(lev)), msg);
#elif MALIPUT_FMT_VERSION == 4
      fmt::print_colored(FromLogLevelToColor(lev), msg);
#endif
    };
  } else {
    print_ = [](const std::string& msg, logger::level) { fmt::print(msg); };
  }
}

void Sink::log(const std::string& msg, logger::level lev) { print_(msg, lev); }

}  // namespace common

common::Logger* log() {
  static common::never_destroyed<std::unique_ptr<common::Logger>> g_logger{std::make_unique<common::Logger>()};
  return g_logger.access().get();
}

}  // namespace maliput
