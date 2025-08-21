#pragma once

// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/common/drake_throw.h
//
// Drake's license follows:
//
// All components of Drake are licensed under the BSD 3-Clause License
// shown below. Where noted in the source code, some portions may
// be subject to other permissive, non-viral licenses.
//
// Copyright 2012-2016 Robot Locomotion Group @ CSAIL
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.  Redistributions
// in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.  Neither the name of
// the Massachusetts Institute of Technology nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

#include <sstream>
#include <string>

#include "maliput/common/error.h"

namespace maliput {
namespace common {
namespace internal {
namespace {

// Stream into @p out the given failure details; only @p condition may be null.
inline void PrintFailureDetailTo(std::ostream* out, const char* condition, const char* func, const char* file,
                                 int line) {
  (*out) << "Failure at " << file << ":" << line << " in " << func << "()";
  if (condition && (std::string(condition).length() > 0)) {
    (*out) << ": condition '" << condition << "' failed.";
  } else {
    (*out) << ".";
  }
}

}  // namespace

// Throw an error message.
template <typename ExceptionType>
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void Throw(const char* condition, const char* func, const char* file, int line) {
  std::ostringstream what;
  PrintFailureDetailTo(&what, condition, func, file, line);
  throw ExceptionType(what.str());
}

}  // namespace internal
}  // namespace common
}  // namespace maliput

/// Evaluates @p condition and iff the value is false will throw an exception
/// with a message showing at least the condition text, function name, file,
/// and line.
#define MALIPUT_THROW_UNLESS(condition)                                                                               \
  do {                                                                                                                \
    if (!(condition)) {                                                                                               \
      ::maliput::common::internal::Throw<maliput::common::assertion_error>(#condition, __func__, __FILE__, __LINE__); \
    }                                                                                                                 \
  } while (0)

/// Throws an exception with a message showing at least the condition text,
/// function name, file, and line.
#define MALIPUT_THROW_MESSAGE(msg)                                                                                  \
  do {                                                                                                              \
    const std::string error_message(msg);                                                                           \
    ::maliput::common::internal::Throw<maliput::common::assertion_error>(error_message.c_str(), __func__, __FILE__, \
                                                                         __LINE__);                                 \
  } while (0)

/// Evaluates @p condition and iff the value is false will throw a
/// road_network_description_parser_error with a message showing at least the
/// condition text, function name, file, and line.
#define MALIPUT_THROW_ROAD_NETWORK_DESCRIPTION_PARSER_UNLESS(condition)                                                \
  do {                                                                                                                 \
    if (!(condition)) {                                                                                                \
      ::maliput::common::internal::Throw<maliput::common::road_network_description_parser_error>(#condition, __func__, \
                                                                                                 __FILE__, __LINE__);  \
    }                                                                                                                  \
  } while (0)

/// Throws a road_network_description_parser_error with a message showing at
/// least the condition text, function name, file, and line.
#define MALIPUT_THROW_ROAD_NETWORK_DESCRIPTION_PARSER_MESSAGE(msg)                              \
  do {                                                                                          \
    const std::string error_message(msg);                                                       \
    ::maliput::common::internal::Throw<maliput::common::road_network_description_parser_error>( \
        error_message.c_str(), __func__, __FILE__, __LINE__);                                   \
  } while (0)

/// Evaluates @p condition and iff the value is false will throw a
/// road_geometry_construction_error with a message showing at least the
/// condition text, function name, file, and line.
#define MALIPUT_THROW_ROAD_GEOMETRY_CONSTRUCTION_UNLESS(condition)                                                \
  do {                                                                                                            \
    if (!(condition)) {                                                                                           \
      ::maliput::common::internal::Throw<maliput::common::road_geometry_construction_error>(#condition, __func__, \
                                                                                            __FILE__, __LINE__);  \
    }                                                                                                             \
  } while (0)

/// Throws a road_geometry_construction_error with a message showing at
/// least the condition text, function name, file, and line.
#define MALIPUT_THROW_ROAD_GEOMETRY_CONSTRUCTION_MESSAGE(msg)                              \
  do {                                                                                     \
    const std::string error_message(msg);                                                  \
    ::maliput::common::internal::Throw<maliput::common::road_geometry_construction_error>( \
        error_message.c_str(), __func__, __FILE__, __LINE__);                              \
  } while (0)

/// @def MALIPUT_VALIDATE
/// Used to validate that a @p pred-icate passed into a function or method is
/// true; if not, an exception with @p message is thrown.
#define MALIPUT_VALIDATE(pred, message)                                                                            \
  do {                                                                                                             \
    if (!(pred)) {                                                                                                 \
      ::maliput::common::internal::Throw<maliput::common::assertion_error>(std::string(message).c_str(), __func__, \
                                                                           __FILE__, __LINE__);                    \
    }                                                                                                              \
  } while (0)

/// @def MALIPUT_IS_IN_RANGE
/// Throws if `value` is within [`min_value`; `max_value`]. It forwards the call
/// to MALIPUT_VALIDATE() with a customized string stating the error.
#define MALIPUT_IS_IN_RANGE(value, min_value, max_value)                                                           \
  do {                                                                                                             \
    MALIPUT_VALIDATE(value >= min_value, std::to_string(value) + " is less than " + std::to_string(min_value));    \
    MALIPUT_VALIDATE(value <= max_value, std::to_string(value) + " is greater than " + std::to_string(max_value)); \
  } while (0)
