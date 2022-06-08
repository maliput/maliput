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

#include "maliput/api/regions.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_deprecated.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

/// Rule describing speed limits.
///
/// Each rule instance describes speed limits applied to a longitudinal
/// portion of a Lane (which may be the entire length of the Lane).  Each
/// instance is tagged with a severity; multiple instances with different
/// severities may apply to the same region of the road network.
///
/// Each instance mandates a maximum speed limit as well as a minimum
/// speed limit.  Since neither limit may be less than zero, a minimum
/// limit of exactly zero is equivalent to having no minimum limit at all.
class MALIPUT_DEPRECATED("next release", "Use RangeValueRule instead.") SpeedLimitRule {
 public:
  using Id = TypeSpecificIdentifier<class SpeedLimitRule>;

  /// Severity classification.
  enum class Severity {
    kStrict = 0,  ///< A strict limit is the established mandatory limit.
    kAdvisory     ///< An advisory limit is a recommendation, typically
                  ///  reflecting a road condition (e.g., a sharp curve).
  };

  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SpeedLimitRule);

  /// Constructs a SpeedLimitRule.
  ///
  /// @param id the unique ID of this rule (in the RoadRulebook)
  /// @param zone LaneSRange to which this rule applies
  /// @param severity Severity of the rule
  /// @param min minimum speed
  /// @param max maximum speed
  ///
  /// `min` and `max` must be non-negative, and `min` must be less than
  /// or equal to `max`, otherwise a maliput::common::assertion_error is thrown.
  SpeedLimitRule(const Id& id, const LaneSRange& zone, Severity severity, double min, double max)
      : id_(id), zone_(zone), severity_(severity), min_(min), max_(max) {
    MALIPUT_VALIDATE(min >= 0., "SpeedLimitRule(" + id_.string() + ") min is negative.");
    MALIPUT_VALIDATE(max >= 0., "SpeedLimitRule(" + id_.string() + ") max is negative.");
    MALIPUT_VALIDATE(min <= max, "SpeedLimitRule(" + id_.string() + ") min is greater than max");
  }

  /// Returns the persistent identifier.
  const Id& id() const { return id_; }

  /// Returns the zone to which this rule instance applies.
  const LaneSRange& zone() const { return zone_; }

  /// Returns the severity of this rule instance.
  Severity severity() const { return severity_; }

  /// Returns the minimum limit.
  double min() const { return min_; }

  /// Returns the maximum limit.
  double max() const { return max_; }

 private:
  Id id_;
  LaneSRange zone_;
  Severity severity_{};
  double min_{};
  double max_{};
  // TODO(maddog@tri.global)  Capture applicable vehicle types (e.g., 'trucks').
  // TODO(maddog@tri.global)  Capture time-of-day aspect (e.g., 'night only')
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
