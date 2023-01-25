// BSD 3-Clause License
//
// Copyright (c) 2023, Woven Planet.
// All rights reserved.
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
#pragma once

/// @file Wraps ignition common's profiler behind maliput macros.
///       This allows us to enable/disable profiling at compile time and avoid adding a dependency on ignition common.

#ifndef MALIPUT_PROFILER_ENABLE
/// Always set this variable to some value
#define MALIPUT_PROFILER_ENABLE 0
#endif

#if MALIPUT_PROFILER_ENABLE

#define IGN_PROFILER_ENABLE 1
#include <ignition/common/Profiler.hh>

/// \brief Set name of profiled thread
#define MALIPUT_PROFILE_THREAD_NAME(name) IGN_PROFILE_THREAD_NAME(name)
/// \brief Log profiling text, if supported by implementation
#define MALIPUT_PROFILE_LOG_TEXT(name) IGN_PROFILE_LOG_TEXT(name)
/// \brief Being profiling sample
#define MALIPUT_PROFILE_BEGIN(name) IGN_PROFILE_BEGIN(name)
/// \brief End profiling sample
#define MALIPUT_PROFILE_END() IGN_PROFILE_END()

/// \brief Convenience wrapper for scoped profiling sample. Use MALIPUT_PROFILE
#define MALIPUT_PROFILE_L(name, line) IGN_PROFILE_L(name, line)
/// \brief Scoped profiling sample. Sample will stop at end of scope.
#define MALIPUT_PROFILE(name) IGN_PROFILE(name)
/// \brief Scoped profiling sample as MALIPUT_PROFILE.
///        __FUNCTION__ is used as the sample name.
#define MALIPUT_PROFILE_FUNC() IGN_PROFILE(__FUNCTION__)
/// \brief Scoped profiling sample as MALIPUT_PROFILE.
///        __PRETTY_FUNCTION__ is used as the sample name.
#define MALIPUT_PROFILE_PRETTY_FUNC() IGN_PROFILE(__PRETTY_FUNCTION__)

/// \brief Macro to determine if profiler is enabled and has an implementation.
#define MALIPUT_PROFILER_VALID IGN_PROFILER_VALID
#else

#define MALIPUT_PROFILE_THREAD_NAME(name) ((void)name)
#define MALIPUT_PROFILE_LOG_TEXT(name) ((void)name)
#define MALIPUT_PROFILE_BEGIN(name) ((void)name)
#define MALIPUT_PROFILE_END() ((void)0)
#define MALIPUT_PROFILE_L(name, line) ((void)name)
#define MALIPUT_PROFILE(name) ((void)name)
#define MALIPUT_PROFILE_FUNC() ((void)0)
#define MALIPUT_PROFILE_PRETTY_FUNC() ((void)0)
#define MALIPUT_PROFILER_VALID MALIPUT_PROFILER_ENABLE

#endif  // MALIPUT_PROFILER_ENABLE
