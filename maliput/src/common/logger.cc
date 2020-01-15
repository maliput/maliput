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

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <utility>

#ifdef HAVE_SPDLOG

#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_sinks.h>

#endif  // HAVE_SPDLOG

#include "maliput/common/maliput_never_destroyed.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {

#ifdef HAVE_SPDLOG

namespace {

// Returns the default logger.
// @note This function assumes that it is mutexed, as in the
// initializer of a static local.
std::shared_ptr<logging::Logger> onetime_create_log() {
  // Check if anyone has already set up a logger named "console".  If so, we
  // will just return it; if not, we'll create our own default one.
  std::shared_ptr<logging::Logger> result(spdlog::get("console"));
  if (!result) {
    // We wrap our stderr sink in a dist_sink so that users can atomically swap
    // out the sinks used by all maliput logging, via dist_sink_mt's APIs.
    auto wrapper = std::make_shared<spdlog::sinks::dist_sink_mt>();
    // We use the stderr_sink_mt (instead of stderr_sink_st) so more than one
    // thread can use this logger and have their messages be staggered by line,
    // instead of co-mingling their character bytes.
    wrapper->add_sink(std::make_shared<spdlog::sinks::stderr_sink_mt>());
    result = std::make_shared<logging::Logger>("console", std::move(wrapper));
    result->set_level(spdlog::level::info);
  }
  return result;
}

}  // namespace

logging::Logger* log() {
  static const maliput::common::never_destroyed<std::shared_ptr<logging::Logger>> g_logger(onetime_create_log());
  return g_logger.access().get();
}

logging::Sink* logging::get_dist_sink() {
  // Extract the dist_sink_mt from Maliput's logger instance.
  auto* sink = log()->sinks().empty() ? nullptr : log()->sinks().front().get();
  auto* result = dynamic_cast<spdlog::sinks::dist_sink_mt*>(sink);
  if (result == nullptr) {
    MALIPUT_THROW_MESSAGE("spdlog sink configuration has unexpectedly changed.");
  }
  return result;
}

std::string logging::set_log_level(const std::string& level) {
  const std::string kUnchanged{"unchanged"};
  const std::unordered_map<std::string, spdlog::level::level_enum> kStringToSpdLogLevel{
      {"trace", spdlog::level::trace}, {"debug", spdlog::level::debug}, {"info", spdlog::level::info},
      {"warn", spdlog::level::warn},   {"err", spdlog::level::err},     {"critical", spdlog::level::critical},
      {"off", spdlog::level::off},
  };

  spdlog::level::level_enum prev_value = maliput::log()->level();
  spdlog::level::level_enum value{};
  if (level == kUnchanged) {
    value = prev_value;
  } else if (kStringToSpdLogLevel.find(level) != kStringToSpdLogLevel.end()) {
    value = kStringToSpdLogLevel.at(level);
  } else {
    MALIPUT_THROW_MESSAGE(fmt::format("Unknown spdlog level: <{}>.", level));
  }

  maliput::log()->set_level(value);
  const auto it = std::find_if(kStringToSpdLogLevel.begin(), kStringToSpdLogLevel.end(),
                               [prev_value](const auto& key_val) { return key_val.second == prev_value; });
  if (it == kStringToSpdLogLevel.end()) {
    // N.B. `spdlog::level::level_enum` is not a `enum class`, so the
    // compiler does not know that it has a closed set of values. For
    // simplicity in linking, we do not use one of the `MALIPUT_ABORT()`
    // options.
    MALIPUT_THROW_MESSAGE(fmt::format("Unknown spdlog previous level: <{}>,", prev_value));
  }
  return it->first;
}

#else  // HAVE_SPDLOG

logging::Logger* log() {
  static logging::Logger g_logger;  // A do-nothing logger instance.
  return &g_logger;
}

logging::Sink* logging::get_dist_sink() {
  static logging::Sink g_sink;  // An empty sink instance.
  return &g_sink;
}

std::string logging::set_log_level(const std::string&) { return ""; }

#endif  // HAVE_SPDLOG

}  // namespace maliput
