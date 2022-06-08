#pragma once

// Code in this file is inspired by:
// https://github.com/RobotLocomotion/drake/blob/master/common/never_destroyed.h
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

#include <new>
#include <type_traits>
#include <utility>

#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace common {

/// Wraps an underlying type T such that its storage is a direct member field
/// of this object (i.e., without any indirection into the heap), but *unlike*
/// most member fields T's destructor is never invoked.
///
/// This is especially useful for function-local static variables that are not
/// trivially destructable.  We shouldn't call their destructor at program exit
/// because of the "indeterminate order of ... destruction" as mentioned in
/// cppguide's
/// <a href="https://drake.mit.edu/styleguide/cppguide.html#Static_and_Global_Variables">Static
/// and Global Variables</a> ** section, but other solutions to this problem place
/// the objects on the heap through an indirection.
///
/// ** This project follows rules listed in https://google.github.io/styleguide/cppguide.html but
/// for the sake of limiting the number of editions to the copy, we delegate the decision
/// to authors and reviewers discretion.
///
/// Compared with other approaches, this mechanism more clearly describes the
/// intent to readers, avoids "possible leak" warnings from memory-checking
/// tools, and is probably slightly faster.
///
/// Example uses:
///
/// The singleton pattern:
/// @code
/// class Singleton {
///  public:
///   MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Singleton)
///   static Singleton& getInstance() {
///     static never_destroyed<Singleton> instance;
///     return instance.access();
///   }
///  private:
///   friend never_destroyed<Singleton>;
///   Singleton() = default;
/// };
/// @endcode
///
/// A lookup table, created on demand the first time its needed, and then
/// reused thereafter:
/// @code
/// enum class Foo { kBar, kBaz };
/// Foo ParseFoo(const std::string& foo_string) {
///   using Dict = std::unordered_map<std::string, Foo>;
///   static const maliput::common::never_destroyed<Dict> string_to_enum{
///     std::initializer_list<Dict::value_type>{
///       {"bar", Foo::kBar},
///       {"baz", Foo::kBaz},
///     }
///   };
///   return string_to_enum.access().at(foo_string);
/// }
/// @endcode
//
// The above examples are repeated in the unit test; keep them in sync.
template <typename T>
class never_destroyed {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(never_destroyed)

  /// Passes the constructor arguments along to T using perfect forwarding.
  template <typename... Args>
  explicit never_destroyed(Args&&... args) {
    // Uses "placement new" to construct a `T` in `storage_`.
    new (&storage_) T(std::forward<Args>(args)...);
  }

  /// Does nothing.  Guaranteed!
  ~never_destroyed() = default;

  /// Returns the underlying T reference.
  T& access() { return *reinterpret_cast<T*>(&storage_); }
  const T& access() const { return *reinterpret_cast<const T*>(&storage_); }

 private:
  typename std::aligned_storage<sizeof(T), alignof(T)>::type storage_;
};

}  // namespace common
}  // namespace maliput
