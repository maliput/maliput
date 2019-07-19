#pragma once

#include <stdexcept>
#include <string>

namespace maliput {
namespace common {

/// This is what MALIPUT_THROW_UNLESS throws.
class assertion_error : public std::runtime_error {
 public:

  /// Constructs an assertion_error with @p what_arg as description.
  explicit assertion_error(const std::string& what_arg) :
      std::runtime_error(what_arg) {}
};

}  // namespace common
}  // namespace maliput
