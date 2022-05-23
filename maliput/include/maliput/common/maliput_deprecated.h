#pragma once

// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/common/drake_deprecated.h
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
//
//
// Provides a portable macro for use in generating compile-time warnings for
// use of code that is permitted but discouraged. */

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

#ifdef BUILD_DOCS

#define MALIPUT_DEPRECATED(...)

#else  // BUILD_DOCS

/// Auxiliary macro declarations for overloading #MALIPUT_DEPRECATED macro. Please use ONLY #MALIPUT_DEPRECATED().
/// @{
#define MALIPUT_DEPRECATED_MSG(message) MALIPUT_DEPRECATED_MSG_REPLACEMENT(message, "None")
#define MALIPUT_DEPRECATED_MSG_REPLACEMENT(message, replacement) \
  MALIPUT_DEPRECATED_MSG_REPLACEMENT_DATE(message, replacement, "unkown")
#define MALIPUT_DEPRECATED_MSG_REPLACEMENT_DATE(message, replacement, removal_date)  \
  [[deprecated("MALIPUT DEPRECATED: " message ". " replacement " is an alternative." \
               " Deprecation date is " removal_date ".")]]

#define MALIPUT_GET_DEPRECATED_MACRO(_1, _2, _3, DEPRECATED_MACRO_NAME, ...) DEPRECATED_MACRO_NAME
/// @}

/// Adds "deprecated" attribute to classes, methods, variables, aliases.
/// _1 : message: Deprecation message.
/// _2 : replacement: Alternative entity suggested to be used.
/// _3 : removal_date: Estimated date for removal.
///
/// Example of use:
/// * Method:
/// @code{cpp}
/// MALIPUT_DEPRECATED("MyOldMethod will be deprecated")
/// int MyOldMethod(int x) {
///   // ...
/// }
/// @endcode
///
/// * Class:
/// @code{cpp}
/// class MALIPUT_DEPRECATED("MyOldClass will be deprecated", "MyNewClass") MyOldClass {
///   // ...
/// };
/// @endcode
///
/// * Variable:
/// @code{cpp}
/// MALIPUT_DEPRECATED("my_old_var will be deprecated")
/// static constexpr int my_old_var = 2;
/// @endcode
///
/// * Type Alias:
/// @code{cpp}
/// using MyOldAlias MALIPUT_DEPRECATED("MyOldAlias will be deprecated.") = std::map<std::string, double>;
/// @endcode
///
#define MALIPUT_DEPRECATED(...)                                                                     \
  MALIPUT_GET_DEPRECATED_MACRO(__VA_ARGS__, MALIPUT_DEPRECATED_MSG_REPLACEMENT_DATE,                \
                               MALIPUT_DEPRECATED_MSG_REPLACEMENT, MALIPUT_DEPRECATED_MSG, _UNUSED) \
  (__VA_ARGS__)

#endif  // BUILD_DOCS
