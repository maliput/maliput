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
#pragma once

#include <functional>
#include <string>
#include <utility>

#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_hash.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {

/// TypeSpecificIdentifier<T> represents an identifier specifically identifying
/// an entity of type `T`.
///
/// A new %TypeSpecificIdentifier is constructed from a non-empty string;
/// %TypeSpecificIdentifiers constructed from equal strings are considered to
/// be equal.  There is currently no other semantic value attributed to the
/// contents of the string.
///
/// Construction from empty strings is not allowed; there is no notion of
/// an "unassigned" value for a %TypeSpecificIdentifier.  To represent a
/// possibly-unassigned %TypeSpecificIdentifier, use
/// std::optional<TypeSpecificIdentifier<T>>.
///
/// %TypeSpecificIdentifier is EqualityComparable (and provides == and !=
/// operators), but it is not LessThanComparable; there is no particular
/// ordering ascribed to %TypeSpecificIdentifier instances.  However,
/// %TypeSpecificIdentifier does provide a strict weak ordering via a
/// specialization of std::less for use in ordered containers such as std::set
/// and std::map.  This ordering may change in future implementations of
/// %TypeSpecificIdentifier.
///
/// %TypeSpecificIdentifier also provides a specialization of std::hash to make
/// it easy to use with std::unordered_set and std::unordered_map.
template <typename T>
class TypeSpecificIdentifier {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TypeSpecificIdentifier);

  /// The type whose instances are identified by this %TypeSpecificIdentifier.
  typedef T identified_type;

  /// Constructs a %TypeSpecificIdentifier from the given `string`.
  ///
  /// @throws maliput::common::assertion_error if `string` is empty.
  explicit TypeSpecificIdentifier(std::string string) : string_(std::move(string)) {
    MALIPUT_THROW_UNLESS(!string_.empty());
  }

  /// Returns the string representation of the %TypeSpecificIdentifier.
  const std::string& string() const { return string_; }

  /// Tests for equality with another %TypeSpecificIdentifier.
  bool operator==(const TypeSpecificIdentifier<T>& rhs) const { return string_ == rhs.string_; }

  /// Tests for inequality with another %TypeSpecificIdentifier, specifically
  /// returning the opposite of operator==().
  bool operator!=(const TypeSpecificIdentifier<T>& rhs) const { return !(*this == rhs); }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const TypeSpecificIdentifier& item) noexcept {
    using maliput::common::hash_append;
    hash_append(hasher, item.string_);
  }

 private:
  std::string string_;
};

}  // namespace api
}  // namespace maliput

namespace std {

/// Specialization of std::hash for maliput::api::TypeSpecificIdentifier<T>.
template <typename T>
struct hash<maliput::api::TypeSpecificIdentifier<T>> : public maliput::common::DefaultHash {};

/// Specialization of std::less for maliput::api::TypeSpecificIdentifier<T>
/// providing a strict ordering over maliput::api::TypeSpecificIdentifier<T>
/// suitable for use with ordered containers.
template <typename T>
struct less<maliput::api::TypeSpecificIdentifier<T>> {
  bool operator()(const maliput::api::TypeSpecificIdentifier<T>& lhs,
                  const maliput::api::TypeSpecificIdentifier<T>& rhs) const {
    return lhs.string() < rhs.string();
  }
};

/// Streams a string representation of @p id into @p out. Returns
/// @p out. This method is provided for the purposes of debugging or
/// text-logging. It is not intended for serialization.
template <typename T>
std::ostream& operator<<(std::ostream& os, const maliput::api::TypeSpecificIdentifier<T>& id) {
  os << id.string();
  return os;
}

}  // namespace std
