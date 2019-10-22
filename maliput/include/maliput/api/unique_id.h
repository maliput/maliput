#pragma once

#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {

/// Represents an unique identifier.
///
/// Uniqueness is not guaranteed by this class. ID providers will be responsible
/// of guaranteeing uniqueness across the specified domain.
class UniqueId {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniqueId);

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
    using drake::hash_append;
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
struct hash<maliput::api::UniqueId> : public drake::DefaultHash {};

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
