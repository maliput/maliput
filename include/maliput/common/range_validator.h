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

#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace common {

/// Functor to validate if a number is within [min, max] and considering
/// assuming a tolerance. This tolerance extends the range to be
/// [min - tolerance, max + tolerance].
/// The functor not only validates that number is within the range, but also
/// adapts it to be in the interval (min, max) where the difference between the
/// close and open range is epsilon.
/// This functor is motivated to wrap MALIPUT_VALIDATE() calls plus a std::clamp() to avoid throws for numerical errors.
class RangeValidator {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RangeValidator)

  /// Creates a RangeValidator instance that uses a relative epsilon to validate the values.
  /// This relative epsilon is computed by multiplying `epsilon` by the range.
  /// @param min lower extreme of the range.
  /// @param max upper extreme of the range.
  /// @param tolerance is the range extension to be accepted.
  /// @param epsilon is the relative minimum difference that separates a number within
  ///                range to be distinct from extremes. It is relative to the range.
  /// @returns A RangeValidator instance.
  /// @throws maliput::common::assertion_error When `tolerance` is non positive.
  /// @throws maliput::common::assertion_error When `epsilon` is not in
  ///         [0, tolerance].
  /// @throws maliput::common::assertion_error When `min` + `epsilon` > `max` or
  ///         `max` - `epsilon` < `min`
  static RangeValidator GetRelativeEpsilonValidator(double min, double max, double tolerance, double epsilon);

  /// Creates a RangeValidator instance that uses a `epsilon` as the absolute epsilon to validate the values.
  /// @param min lower extreme of the range.
  /// @param max upper extreme of the range.
  /// @param tolerance is the range extension to be accepted.
  /// @param epsilon is minimum difference that separates a number within the
  ///        range to be distinct from range extremes (`min` and `max`).
  /// @returns A RangeValidator instance.
  /// @throws maliput::common::assertion_error When `tolerance` is non positive.
  /// @throws maliput::common::assertion_error When `epsilon` is not in
  ///         [0, tolerance].
  /// @throws maliput::common::assertion_error When `min` + `epsilon` > `max` or
  ///         `max` - `epsilon` < `min`
  static RangeValidator GetAbsoluteEpsilonValidator(double min, double max, double tolerance, double epsilon);

  RangeValidator() = delete;

  /// Evaluates whether `s` is in range or not.
  /// @returns `s` when it is within the open range. If `s` is equal to either
  /// range extremes or the difference to them is less or equal to tolerance, it
  /// returns the closest open range value. Otherwise, it
  /// @throws maliput::common::assertion_error.
  double operator()(double s) const;

 private:
  // The provided epsilon can be used relatively or absolutely.
  enum class EpsilonUse { kAbsolute = 0, kRelative };

  // Constructs the functor.
  //
  // @param min lower extreme of the range.
  // @param max upper extreme of the range.
  // @param tolerance is the range extension to be accepted.
  // @param epsilon is minimum difference that separates a number within the
  //        range to be distinct from range extremes (`min` and `max`).
  // @param epsilon_mode selects how `epsilon` will be used: absolute or relative.
  // @throws maliput::common::assertion_error When `tolerance` is non positive.
  // @throws maliput::common::assertion_error When `epsilon` is not in
  //         [0, tolerance].
  // @throws maliput::common::assertion_error When `min` + `epsilon` > `max` or
  //         `max` - `epsilon` < `min`
  RangeValidator(double min, double max, double tolerance, double epsilon, const EpsilonUse& epsilon_mode);

  double min_{};
  double max_{};
  double tolerance_{};
  double epsilon_{};
};

}  // namespace common
}  // namespace maliput
