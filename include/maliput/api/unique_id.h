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

#include <string>

#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_hash.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {

/// Represents an unique identifier.
///
/// Uniqueness is not guaranteed by this class. ID providers will be responsible
/// of guaranteeing uniqueness across the specified domain.
class UniqueId {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniqueId);

  /// Constructs a UniqueId from the given `string`.
  ///
  /// @throws maliput::common::assertion_error if `string` is empty.
  explicit UniqueId(const std::string& string) : string_(string) { MALIPUT_THROW_UNLESS(!string_.empty()); }

  /// Returns the string representation of the UniqueId.
  const std::string& string() const { return string_; }

  /// Tests for equality with another UniqueId.
  bool operator==(const UniqueId& rhs) const { return string_ == rhs.string_; }

  /// Tests for inequality with another UniqueId, specifically returning the
  /// opposite of operator==().
  bool operator!=(const UniqueId& rhs) const { return !(*this == rhs); }

  /// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const UniqueId& item) noexcept {
    using maliput::common::hash_append;
    hash_append(hasher, item.string_);
  }

 private:
  std::string string_;
};

}  // namespace api
}  // namespace maliput

namespace std {

/// Specialization of std::hash for maliput::api::UniqueId.
template <>
struct hash<maliput::api::UniqueId> : public maliput::common::DefaultHash {};

/// Specialization of std::less for maliput::api::UniqueId providing a
/// strict ordering over UniqueId suitable for use with ordered
/// containers.
template <>
struct less<maliput::api::UniqueId> {
  bool operator()(const maliput::api::UniqueId& lhs, const maliput::api::UniqueId& rhs) const {
    return lhs.string() < rhs.string();
  }
};

}  // namespace std
