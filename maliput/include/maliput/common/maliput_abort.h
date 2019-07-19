#pragma once

namespace maliput {
namespace common {
namespace internal {

// Abort the program with an error message.
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void Abort(const char* condition, const char* func, const char* file, int line);

}  // namespace internal
}  // namespace common
}  // namespace maliput


/// Evaluates @p condition and iff the value is false will trigger an abortion
/// with a message showing at least the condition text, function name, file, and
/// line.
#define MALIPUT_DEMAND(condition)                                             \
  do {                                                                        \
    if (!condition) {                                                         \
      ::maliput::common::internal::Abort(                                     \
          #condition, __func__, __FILE__, __LINE__);                          \
    }                                                                         \
  } while (0)

