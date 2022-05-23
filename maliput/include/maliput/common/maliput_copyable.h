#pragma once

// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/common/drake_copyable.h
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

/** @file
Provides careful macros to selectively enable or disable the special member
functions for copy-construction, copy-assignment, move-construction, and
move-assignment.
http://en.cppreference.com/w/cpp/language/member_functions#Special_member_functions
When enabled via these macros, the `= default` implementation is provided.
Code that needs custom copy or move functions should not use these macros.
*/

/** MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN deletes the special member functions for
copy-construction, copy-assignment, move-construction, and move-assignment.
Invoke this this macro in the public section of the
class declaration, e.g.:
@code{cpp}
class Foo {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Foo)
  // ...
};
@endcode
*/
#define MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Classname) \
  Classname(const Classname&) = delete;              \
  void operator=(const Classname&) = delete;         \
  Classname(Classname&&) = delete;                   \
  void operator=(Classname&&) = delete;

/** MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN defaults the special member
functions for copy-construction, copy-assignment, move-construction, and
move-assignment.  This macro should be used only when copy-construction and
copy-assignment defaults are well-formed.  Note that the defaulted move
functions could conceivably still be ill-formed, in which case they will
effectively not be declared or used -- but because the copy constructor exists
the type will still be MoveConstructible. Invoke this this
macro in the public section of the class declaration, e.g.:
@code{cpp}
class Foo {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Foo)
  // ...
};
@endcode
*/
#define MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Classname)                                \
  Classname(const Classname&) = default;                                                   \
  Classname& operator=(const Classname&) = default;                                        \
  Classname(Classname&&) = default;                                                        \
  Classname& operator=(Classname&&) = default;                                             \
  /* Fails at compile-time if default-copy doesn't work. */                                \
  static void MALIPUT_COPYABLE_DEMAND_COPY_CAN_COMPILE() {                                 \
    (void)static_cast<Classname& (Classname::*)(const Classname&)>(&Classname::operator=); \
  }

/** MALIPUT_DECLARE_COPY_AND_MOVE_AND_ASSIGN declares (but does not define) the
special member functions for copy-construction, copy-assignment,
move-construction, and move-assignment.
This is useful when paired with MALIPUT_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T
to work around https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 whereby the
declaration and definition must be split.  Once Maliput no longer supports GCC
versions prior to 6.3, this macro could be removed.
Invoke this macro in the public section of the class declaration, e.g.:
@code{cpp}
template <typename T>
class Foo {
 public:
  MALIPUT_DECLARE_COPY_AND_MOVE_AND_ASSIGN(Foo)
  // ...
};
MALIPUT_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T(Foo)
@endcode
*/
#define MALIPUT_DECLARE_COPY_AND_MOVE_AND_ASSIGN(Classname)                                \
  Classname(const Classname&);                                                             \
  Classname& operator=(const Classname&);                                                  \
  Classname(Classname&&);                                                                  \
  Classname& operator=(Classname&&);                                                       \
  /* Fails at compile-time if default-copy doesn't work. */                                \
  static void MALIPUT_COPYABLE_DEMAND_COPY_CAN_COMPILE() {                                 \
    (void)static_cast<Classname& (Classname::*)(const Classname&)>(&Classname::operator=); \
  }

/** Helper for MALIPUT_DECLARE_COPY_AND_MOVE_AND_ASSIGN.  Provides defaulted
definitions for the four special member functions of a templated class.
*/
#define MALIPUT_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T(Classname)    \
  template <typename T>                                                 \
  Classname<T>::Classname(const Classname<T>&) = default;               \
  template <typename T>                                                 \
  Classname<T>& Classname<T>::operator=(const Classname<T>&) = default; \
  template <typename T>                                                 \
  Classname<T>::Classname(Classname<T>&&) = default;                    \
  template <typename T>                                                 \
  Classname<T>& Classname<T>::operator=(Classname<T>&&) = default;
