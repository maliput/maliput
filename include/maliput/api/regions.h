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
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

#include "maliput/api/lane.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {

class RoadGeometry;

/// Directed, inclusive longitudinal (s value) range from s0 to s1.
class SRange {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SRange);

  /// Default constructor:  initializes s0 and s1 to zero.
  SRange() = default;

  /// Constructs range [s0 --> s1].
  ///
  /// @param s0 The start value of the range. It must not be negative.
  /// @param s1 The end value of the range. It must not be negative.
  /// @throws common::assertion_error When 's0' is negative.
  /// @throws common::assertion_error When 's1' is negative.
  SRange(double s0, double s1);

  /// Gets s0 value.
  ///
  /// @return s0.
  double s0() const { return s0_; }

  /// Gets s1 value.
  ///
  /// @return s1.
  double s1() const { return s1_; }

  /// Sets s0 value.
  ///
  /// @param s0 The start value of the range. It must not be negative.
  /// @throws common::assertion_error When 's0' is negative.
  void set_s0(double s0);

  /// Sets s1 value.
  ///
  /// @param s1 The end value of the range. It must not be negative.
  /// @throws common::assertion_error When 's1' is negative.
  void set_s1(double s1);

  /// Computes the size of this SRange (i.e., |s1() - s0()|).
  ///
  /// @return The size of of this SRange.
  double size() const { return std::fabs(s1() - s0()); }

  /// Defines whether this SRange is in the direction of +s (i.e., s1() > s0()).
  ///
  /// @return true When this SRange is in the direction of +s.
  bool WithS() const { return s1() > s0(); }

  /// Determines whether this SRange intersects with @p s_range.
  ///
  /// @p tolerance will modify this range and @p s_range by increasing the maximum
  /// and reducing the minimum of each range. When @p tolerance is negative, it shrinks both ranges.
  ///
  /// @param s_range A SRange to evaluate the intersection with.
  /// @param tolerance The tolerance to use when computing the intersection.
  /// @return true When @p s_range intersects this SRange.
  bool Intersects(const SRange& s_range, double tolerance) const;

  /// Determines whether this SRange contains @p s_range.
  ///
  /// @p tolerance will modify this range and @p s_range by increasing the maximum
  /// and reducing the minimum of each range. When @p tolerance is negative, it shrinks both ranges.
  ///
  /// @param s_range The SRange with which to compute the intersection.
  /// @param tolerance The tolerance to use when computing the inclusion relationship.
  /// @return true When @p s_range is contained within this SRange.
  bool Contains(const SRange& s_range, double tolerance) const;

  /// Computes a std::optional<SRange> bearing the intersected SRange that results overlapping
  /// this SRange with @p s_range. When there is no common area, std::nullopt is returned.
  ///
  /// @p tolerance will modify this range and @p s_range by increasing the maximum
  /// and reducing the minimum of each range. When @p tolerance is negative, it shrinks both ranges.
  ///
  /// @param s_range The SRange to evaluate whether it is included into this SRange.
  /// @param tolerance The tolerance to use to evaluate the intersection.
  /// @return A std::optional<SRange> with the intersected SRange.
  std::optional<SRange> GetIntersection(const SRange& s_range, double tolerance) const;

 private:
  double s0_{0.};
  double s1_{0.};
};

/// Directed longitudinal range of a specific Lane, identified by a LaneId.
class LaneSRange {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneSRange);

  /// Constructs a LaneSRange as @p s_range on Lane @p lane_id.
  ///
  /// @param lane_id Refers to the Lane this LaneSRange belongs to.
  /// @param s_range The SRange in the @p lane_id Lane.
  LaneSRange(const LaneId& lane_id, const SRange& s_range) : lane_id_(lane_id), s_range_(s_range) {}

  /// Gets the LaneId.
  ///
  /// @return The LaneId.
  const LaneId& lane_id() const { return lane_id_; }

  /// Gets the SRange.
  ///
  /// @return The SRange.
  SRange s_range() const { return s_range_; }

  /// Returns the length of this LaneSRange, which is dictated by the SRange.
  ///
  /// @return The SRange's size.
  double length() const { return s_range_.size(); }

  /// Determines whether this LaneSRange intersects with @p lane_s_range.
  /// LaneIds are evaluated prior calling SRange::Intersects() method.
  ///
  /// @p tolerance will modify this LaneSRanges's ranges and @p lane_s_range 's ranges by increasing the maximum
  /// and reducing the minimum of each range. When @p tolerance is negative, it shrinks both ranges.
  ///
  /// @param lane_s_range The LaneSRange to compute the intersection with.
  /// @param tolerance The tolerance to use to evaluate the intersection.
  /// @return true When @p lane_s_range intersects this LaneSRange.
  bool Intersects(const LaneSRange& lane_s_range, double tolerance) const;

  /// Determines whether this LaneSRange contains @p lane_s_range.
  ///
  /// @p tolerance will modify this LaneSRanges's ranges and @p lane_s_range 's ranges by increasing the maximum
  /// and reducing the minimum of each range. When @p tolerance is negative, it shrinks both ranges.
  ///
  /// @param lane_s_range The LaneSRange to evaluate whether it is included into this LaneSRange.
  /// @param tolerance The tolerance to use to evaluate the inclusion with.
  /// @return true When @p lane_s_range is contained within this LaneSRange.
  bool Contains(const LaneSRange& lane_s_range, double tolerance) const;

  /// Computes a std::optional<LaneSRange> bearing the intersected LaneSRange that results overlapping
  /// this LaneSRange with @p lane_s_range. When there is no common area, std::nullopt is returned.
  ///
  /// @p tolerance will modify this range and @p lane_s_range by increasing the maximum
  /// and reducing the minimum of each range. When @p tolerance is negative, it shrinks both ranges.
  ///
  /// @param lane_s_range The LaneSRange to compute the intersection with.
  /// @param tolerance The tolerance to use to evaluate the intersection.
  /// @return A std::optional<LaneSRange> with the intersected LaneSRange.
  std::optional<LaneSRange> GetIntersection(const LaneSRange& lane_s_range, double tolerance) const;

 private:
  LaneId lane_id_;
  SRange s_range_;
};

/// A route, possibly spanning multiple (end-to-end) lanes.
///
/// The sequence of LaneSRanges should be contiguous by either presenting
/// laterally adjacent LaneSRanges, or consecutive LaneSRanges. (In other words,
/// taken as a Lane-space path with r=0 and h=0, it should present a
/// G1-continuous curve.)
// TODO(#543): Review this class semantics to express the relationship between
// LaneSRanges (laterally adjacent or consecutive).
class LaneSRoute {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LaneSRoute);

  /// Default constructor:  constructs an empty route.
  LaneSRoute() = default;

  /// Constructs a LaneSRoute from the given sequence of LaneSRanges.
  ///
  /// @param ranges A sequence of LaneSRanges.
  explicit LaneSRoute(const std::vector<LaneSRange>& ranges) : ranges_(ranges) {}

  /// Returns the sequence of LaneSRanges.
  ///
  /// @return A vector of LaneSRanges.
  const std::vector<LaneSRange>& ranges() const { return ranges_; }

  /// Computes the accumulated length of all LaneSRanges.
  ///
  /// @return The accumulated length of all LaneSRanges.
  double length() const {
    return std::accumulate(ranges_.cbegin(), ranges_.cend(), 0.,
                           [](const double acc, const LaneSRange& lane_range) { return acc + lane_range.length(); });
  }

  /// Determines whether this LaneSRoute intersects with @p lane_s_route.
  ///
  /// LaneSRoutes intersection is evaluated first by LaneSRange's LaneId coincidence, then LaneSRange::Intersects() is
  /// used.
  ///
  /// @p tolerance will modify this LaneSRoute's ranges and @p lane_s_route 's ranges by increasing the maximum
  /// and reducing the minimum of each range. When @p tolerance is negative, it shrinks both ranges.
  ///
  /// @param lane_s_route The LaneSRoute to compute the intersection with.
  /// @param tolerance The tolerance to use to evaluate the intersection.
  /// @return true When @p lane_s_route intersects this LaneSRoute.
  bool Intersects(const LaneSRoute& lane_s_route, double tolerance) const;

  // TODO(maddog@tri.global)  Implement a "CheckInvariants()" method which
  //                          ensures adjacency (with respect to a specified
  //                          RoadGeometry).

 private:
  std::vector<LaneSRange> ranges_;
};

/// Evaluates whether @p lane_range_a end point is G1 contiguous with @p lane_range_b start pose.
///
/// @param lane_range_a A LaneSRange in @p road_geometry. Its ID must belong to a @p road_geometry 's
/// Lane.
/// @param lane_range_b A LaneSRange in @p road_geometry. Its ID must belong to a @p road_geometry 's
/// Lane.
/// @param road_geometry The RoadGeometry where @p lane_range_a and @p lane_range_b are contained. It must not
/// be nullptr.
/// @returns true When @p lane_range_a  end pose and @p lane_range_b start pose are within linear and
/// angular tolerance in the `Inertial`-frame. Otherwise, it returns false.
///
/// @throws common::assertion_error When @p lane_range_a 's Lane is not found in @p road_geometry.
/// @throws common::assertion_error When @p lane_range_b 's Lane is not found in @p road_geometry.
/// @throws common::assertion_error When @p road_geometry is nullptr.
bool IsContiguous(const LaneSRange& lane_range_a, const LaneSRange& lane_range_b, const RoadGeometry* road_geometry);

/// Evaluates whether @p inertial_position is within @p lane_s_ranges.
///
/// @param inertial_position A InertialPosition in the `Inertial`-frame.
/// @param lane_s_ranges A vector of LaneSRanges that define a region.
/// @param road_geometry The RoadGeometry where @p lane_s_ranges is contained. It must not be nullptr.
/// @returns True when @p inertial_position is within @p lane_s_ranges. @p inertial_position is contained if the
/// distance to the closest LanePosition of @p lane_s_ranges is less or equal than the linear tolerance of the
/// @p road_geometry.
///
/// @throws common::assertion_error When @p road_geometry is nullptr.
/// @throws common::assertion_error When Lanes in @p lane_s_ranges are not found in @p road_geometry.
bool IsIncluded(const InertialPosition& inertial_position, const std::vector<LaneSRange>& lane_s_ranges,
                const RoadGeometry* road_geometry);

}  // namespace api
}  // namespace maliput
