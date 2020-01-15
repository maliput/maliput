#pragma once

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/common/maliput_unused.h"

namespace maliput {
namespace api {
namespace rules {
namespace test {

// TODO(maddog@tri.global) Make the general-purpose assertion collection
//                         machinery available (and used) by maliput in general,
//                         or even drake.

/// AssertionResultCollector helps with the creation of concise and well-traced
/// testing subroutines when using googletest.
///
/// Instead of invocations of `EXPECT_*()` or `ASSERT_*()` within a test
/// subroutine, predicate-assertion functions are invoked and the resulting
/// \::testing\::AssertionResult instances should be collected by an instance
/// of AssertionResultCollector.  At the end of the subroutine, an
/// AssertionResult, representing the `and` of all the collected results,
/// can be extracted from the collector.
class AssertionResultCollector {
 public:
  /// Constructs an empty AssertionResultCollector.
  AssertionResultCollector() = default;

  /// Adds an AssertionResult to the collector.  Typically this is not called
  /// directly, but is invoked via the `ADD_RESULT()` macro.
  ///
  /// `result` is the AssertionResult to be collected.  `expression` is a
  /// printable representation of the expression which yielded the result.
  /// `filename` and `line`, as would be produced by __FILE__ and __LINE__
  /// preprocessor macros, identify the location of the expression which
  /// yielded `result`.
  void AddResult(const char* filename, int line, const char* expression, ::testing::AssertionResult result) {
    ++count_;
    if (!result) {
      ++failed_;
      failure_message_ = failure_message_ + filename + ":" + std::to_string(line) + ": Failure #" +
                         std::to_string(failed_) + ":\n" + "Expression '" + expression + "' failed:\n" +
                         result.message() + "\n";
    }
  }

  /// Returns an AssertionResult reflecting the current state of the
  /// collector, which is basically an `and` of the collected results.
  ::testing::AssertionResult result() {
    if (failed_) {
      return ::testing::AssertionFailure() << failed_ << " of " << count_ << " expressions failed:\n"
                                           << failure_message_;
    } else {
      return ::testing::AssertionSuccess() << count_ << " expressions all succeeded.";
    }
  }

  /// Returns the number of results collected.
  int count() const { return count_; }

  /// Returns the number of failure results collected.
  int failed() const { return failed_; }

 private:
  int count_{0};
  int failed_{0};
  std::string failure_message_;
};

/// Adds AssertionResult `result` to AssertionResultCollector `collector`.
/// The location of the invocation and the literal expression of `result`
/// will be recorded by the collector.
#define MALIPUT_ADD_RESULT(collector, result) collector.AddResult(__FILE__, __LINE__, #result, result)

/// Returns an AssertionResult which is successful if `e1` equals `e2`
/// according to the `IsEqual()` predicate-formatter function.  The
/// literal expressions for `e1` and `e2` will be provided to `IsEqual()`.
#define MALIPUT_IS_EQUAL(e1, e2) ::maliput::api::rules::test::IsEqual(#e1, #e2, e1, e2)

// TODO(maddog@tri.global)  Create macros (like below) as an alternative
//                          to EXPECT_PRED_FORMAT*()/etc, which simply returns
//                          the AssertionResult instead of expecting/asserting.
// #define EVAL_PRED_FORMAT(pred_format, v1, v2) pred_format(#v1, #v2, v1, v2)

// TODO(maddog@tri.global)  All the CmpHelperEQ based IsEqual() methods below
//                          should be folded into a single template that knows
//                          what to do for types with operator==.

/// Predicate-formatter which tests equality of double.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const double& a,
                                          const double& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of TypeSpecificIdentifier<T>.
template <class T>
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression,
                                          const TypeSpecificIdentifier<T>& a, const TypeSpecificIdentifier<T>& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of GeoPosition.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const GeoPosition& a,
                                          const GeoPosition& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a, b);
}

/// Predicate-formatter which tests equality of Rotation.
inline ::testing::AssertionResult IsEqual(const char* a_expression, const char* b_expression, const Rotation& a,
                                          const Rotation& b) {
  return ::testing::internal::CmpHelperEQ(a_expression, b_expression, a.matrix(), b.matrix());
}

}  // namespace test
}  // namespace rules
}  // namespace api
}  // namespace maliput
