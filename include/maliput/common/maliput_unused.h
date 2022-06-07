#pragma once

// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/common/unused.h
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

/// Documents the argument(s) as unused, placating GCC's -Wunused-parameter
/// warning.  This can be called within function bodies to mark that certain
/// parameters are unused.
///
/// When possible, removing the unused parameter is better than placating the
/// warning.  However, in some cases the parameter is part of a virtual API or
/// template concept that is used elsewhere, so we can't remove it.  In those
/// cases, this function might be an appropriate work-around.
///
/// Here's rough advice on how to fix Wunused-parameter warnings:
///
/// (1) If the parameter can be removed entirely, prefer that as the first
///     choice.  (This may not be possible if, e.g., a method must match some
///     virtual API or template concept.)
///
/// (2) Unless the parameter name has acute value, prefer to omit the name of
///     the parameter, leaving only the type, e.g.
/// @code
/// void Print(const State& state) override { /* No state to print. */ }
/// @endcode
///     changes to
/// @code
/// void Print(const State&) override { /* No state to print. */}
/// @endcode
///     This no longer triggers the warning and further makes it clear that a
///     parameter required by the API is definitively unused in the function.
///
///     This is an especially good solution in the context of method
///     definitions (vs declarations); the parameter name used in a definition
///     is entirely irrelevant to Doxygen and most readers.
///
/// (3) When leaving the parameter name intact has acute value, it is
///     acceptable to keep the name and mark it `unused`.  For example, when
///     the name appears as part of a virtual method's base class declaration,
///     the name is used by Doxygen to document the method, e.g.,
/// @code
/// /** Sets the default State of a System.  This default implementation is to
///     set all zeros.  Subclasses may override to use non-zero defaults.  The
///     custom defaults may be based on the given @p context, when relevant.  */
/// virtual void SetDefault(const Context<T>& context, State<T>* state) const {
///   unused(context);
///   state->SetZero();
/// }
/// @endcode
///
template <typename... Args>
void unused(const Args&...) {}

}  // namespace common
}  // namespace maliput
