// BSD 3-Clause License
//
// Copyright (c) 2023, Woven by Toyota. All rights reserved.
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

#include <optional>
#include <string>

namespace maliput {
namespace common {

/// @brief ComparisonResult is a struct that holds the result of a comparison
///        between two objects of type T.
///         - If the objects are equal, then message is empty.
///         - If the objects are not equal, then message contains a string
///           describing the difference.
///        Used as return type by compare methods in maliput.
/// @tparam T The type of the objects being compared.
/// @returns A ComparisonResult<T> object.
template <typename T>
struct ComparisonResult {
  std::optional<std::string> message;
};

/// @brief ComparisonResultCollector is a class that collects the results of a series
///       of comparisons between objects of type T.
class ComparisonResultCollector {
 public:
  ComparisonResultCollector() = default;

  template <typename T>
  void AddResult(const char* filename, int line, const char* expression, ComparisonResult<T> result) {
    ++count_;
    if (result.message.has_value()) {
      ++failed_;
      failure_message_ = failure_message_ + filename + ":" + std::to_string(line) + ": Failure #" +
                         std::to_string(failed_) + ":\n" + "Expression '" + expression + "' failed:\n" +
                         result.message.value() + "\n";
    }
  }
  std::optional<std::string> result() const {
    if (failed_ > 0) {
      return std::to_string(failed_) + " out of " + std::to_string(count_) + " comparisons failed:\n" +
             failure_message_;
    }
    return std::nullopt;
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

#define MALIPUT_ADD_RESULT(collector, result) collector.AddResult(__FILE__, __LINE__, #result, result)

}  // namespace common
}  // namespace maliput
