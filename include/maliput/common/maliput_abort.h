#pragma once

// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/common/drake_assert.h
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

namespace maliput {
namespace common {
namespace internal {

// Abort the program with an error message.
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void Abort(const char* condition, const char* func, const char* file, int line);

// Abort the program with an error message and accepts @p extra_details.
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void Abort(const char* condition, const char* func, const char* file, int line,
           const char* extra_details);

}  // namespace internal
}  // namespace common
}  // namespace maliput

/// Evaluates @p condition and iff the value is false will trigger an abortion
/// with a message showing at least the condition text, function name, file, and
/// line.
#define MALIPUT_DEMAND(condition)                                                   \
  do {                                                                              \
    if (!(condition)) {                                                             \
      ::maliput::common::internal::Abort(#condition, __func__, __FILE__, __LINE__); \
    }                                                                               \
  } while (0)

/// Triggers an abortion with a message showing at least the condition text,
/// function name, file, line and @p msg.
#define MALIPUT_ABORT_MESSAGE(msg)                                              \
  do {                                                                          \
    ::maliput::common::internal::Abort("", __func__, __FILE__, __LINE__, #msg); \
  } while (0)
