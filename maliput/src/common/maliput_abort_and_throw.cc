#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_throw.h"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#include "maliput/common/assertion_error.h"

namespace maliput {
namespace common {
namespace internal {
namespace {

// Stream into @p out the given failure details; only @p condition may be null.
void PrintFailureDetailTo(std::ostream* out, const char* condition,
                          const char* func, const char* file, int line) {
  (*out) << "Failure at " << file << ":" << line << " in " << func << "()";
  if (condition) {
    (*out) << ": condition '" << condition << "' failed.";
  } else {
    (*out) << ".";
  }
}

}  // namespace

// Declared in maliput_abort.h.
void Abort(const char* condition, const char* func, const char* file,
           int line) {
  std::cerr << "abort: ";
  PrintFailureDetailTo(&std::cerr, condition, func, file, line);
  std::cerr << std::endl;
  std::abort();
}

// Declared in maliput_abort.h.
void Abort(const char* condition, const char* func, const char* file,
           int line, const char* extra_details) {
  std::cerr << "abort: ";
  PrintFailureDetailTo(&std::cerr, condition, func, file, line);
  std::cerr << " Details: " << extra_details << std::endl;
  std::abort();
}

// Declared in maliput_throw.h.
void Throw(const char* condition, const char* func, const char* file,
           int line) {
  std::ostringstream what;
  PrintFailureDetailTo(&what, condition, func, file, line);
  throw assertion_error(what.str().c_str());
}

}  // namespace internal
}  // namespace common
}  // namespace maliput
