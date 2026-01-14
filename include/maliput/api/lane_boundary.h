// BSD 3-Clause License
//
// Copyright (c) 2025-2026, Woven by Toyota. All rights reserved.
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
#include <vector>

#include "maliput/api/lane_marking.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {

// Forward declarations.
class Lane;
class Segment;

/// Persistent identifier for a LaneBoundary element.
using LaneBoundaryId = TypeSpecificIdentifier<class LaneBoundary>;

/// @addtogroup lane_marking
/// @{

/// Represents a boundary between adjacent lanes or at the edge of a Segment.
///
/// A LaneBoundary is owned by a Segment and serves as the interface between
/// two adjacent lanes, or between a lane and the segment edge. For a Segment
/// with N lanes, there are N+1 boundaries:
///
/// ```
///                                                        +r direction
///                                                      <─────────────
///   Boundary N    ...    Boundary 2    Boundary 1    Boundary 0
///      |                     |             |             |
///      | Lane N-1  |   ...   |   Lane 1    |   Lane 0    |
///      |                     |             |             |
///  (left edge)                                     (right edge)
/// ```
///
/// Where:
/// - Boundary 0 is the right edge (minimum r coordinate).
/// - Boundary N is the left edge (maximum r coordinate).
/// - Boundaries are indexed with increasing r direction.
///
/// Each LaneBoundary provides:
/// - Reference to the lane on its left (if any).
/// - Reference to the lane on its right (if any).
/// - Query methods for lane markings at specific s-coordinates.
///
/// The design ensures that adjacent lanes share the same boundary object,
/// avoiding redundancy and ensuring consistency. For example, Lane 1's right
/// boundary is the same object as Lane 0's left boundary (Boundary 1).
///
/// This design is influenced by OpenDRIVE's road marking model and OSI's
/// lane boundary concept.
class LaneBoundary {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(LaneBoundary)

  /// Unique identifier for a LaneBoundary.
  using Id = TypeSpecificIdentifier<class LaneBoundary>;

  virtual ~LaneBoundary() = default;

  /// @returns The unique identifier for this LaneBoundary.
  Id id() const { return do_id(); }

  /// @returns The Segment to which this LaneBoundary belongs.
  const Segment* segment() const { return do_segment(); }

  /// @returns The index of this boundary within the parent Segment.
  ///
  /// Boundaries are indexed from 0 (rightmost, minimum r) to num_lanes()
  /// (leftmost, maximum r).
  int index() const { return do_index(); }

  /// @returns The Lane immediately to the left of this boundary (increasing r direction),
  ///          or nullptr if this is the leftmost boundary of the Segment.
  const Lane* lane_to_left() const { return do_lane_to_left(); }

  /// @returns The Lane immediately to the right of this boundary (decreasing r direction),
  ///          or nullptr if this is the rightmost boundary of the Segment.
  const Lane* lane_to_right() const { return do_lane_to_right(); }

  /// Gets the lane marking at a specific s-coordinate along this boundary.
  ///
  /// @param s The s-coordinate along the boundary (in the lane coordinate system).
  ///          Typically in the range [0, segment_length].
  ///
  /// @returns The lane marking result at the specified position, including the
  ///          marking details and the s-range over which it is valid. Returns
  ///          std::nullopt if no marking information is available at that location.
  ///
  /// @note The s-coordinate system follows the lane(s) adjacent to this boundary.
  ///       For consistency, implementations should use the reference lane's
  ///       s-coordinate (e.g., the lane to the right, or lane to the left if
  ///       there's no lane to the right).
  std::optional<LaneMarkingResult> GetMarking(double s) const { return DoGetMarking(s); }

  /// Gets all lane markings along this boundary.
  ///
  /// @returns A vector of LaneMarkingResult, each describing a marking and
  ///          the s-range over which it is valid. The results are ordered by
  ///          increasing s_start. If no markings are available, returns an
  ///          empty vector.
  ///
  /// @note The returned ranges should cover the entire boundary length without
  ///       gaps, though some ranges may have `type == kNone` to indicate
  ///       sections without visible markings.
  std::vector<LaneMarkingResult> GetMarkings() const { return DoGetMarkings(); }

  /// Gets lane markings within a specified s-range.
  ///
  /// @param s_start Start of the s-range to query [m].
  /// @param s_end End of the s-range to query [m].
  ///
  /// @returns A vector of LaneMarkingResult for markings that overlap with
  ///          the specified range. Results are ordered by increasing s_start.
  ///          If no markings are available in the range, returns an empty vector.
  ///
  /// @throws maliput::common::assertion_error if s_start > s_end.
  std::vector<LaneMarkingResult> GetMarkings(double s_start, double s_end) const {
    return DoGetMarkings(s_start, s_end);
  }

 protected:
  LaneBoundary() = default;

 private:
  /// @name NVI (Non-Virtual Interface) implementations.
  /// @{

  /// @returns The unique identifier for this LaneBoundary.
  virtual Id do_id() const = 0;

  /// @returns The Segment to which this LaneBoundary belongs.
  virtual const Segment* do_segment() const = 0;

  /// @returns The index of this boundary within the parent Segment.
  virtual int do_index() const = 0;

  /// @returns The Lane immediately to the left of this boundary,
  ///          or nullptr if this is the leftmost boundary.
  virtual const Lane* do_lane_to_left() const = 0;

  /// @returns The Lane immediately to the right of this boundary,
  ///          or nullptr if this is the rightmost boundary.
  virtual const Lane* do_lane_to_right() const = 0;

  /// @returns The lane marking result at the specified s-coordinate, or std::nullopt.
  virtual std::optional<LaneMarkingResult> DoGetMarking(double s) const = 0;

  /// @returns All lane markings along this boundary.
  virtual std::vector<LaneMarkingResult> DoGetMarkings() const = 0;

  /// @returns Lane markings within the specified s-range.
  virtual std::vector<LaneMarkingResult> DoGetMarkings(double s_start, double s_end) const = 0;

  /// @}
};

/// @}

}  // namespace api
}  // namespace maliput
