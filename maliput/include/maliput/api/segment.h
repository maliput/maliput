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

#include <string>

#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {

class Junction;
class Lane;

/// Persistent identifier for a Segment element.
using SegmentId = TypeSpecificIdentifier<class Segment>;

/// A Segment represents a bundle of adjacent Lanes which share a
/// continuously traversable road surface.  Every LanePosition on a
/// given Lane of a Segment has a corresponding LanePosition on each
/// other Lane, all with the same height-above-surface h, that all
/// map to the same GeoPoint in 3-space.
///
/// Segments are grouped by Junction.
class Segment {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Segment)

  virtual ~Segment() = default;

  /// Returns the persistent identifier.
  SegmentId id() const { return do_id(); }

  /// Returns the Junction to which this Segment belongs.
  const Junction* junction() const { return do_junction(); }

  /// Returns the number of Lanes contained in this Segment.
  ///
  /// Return value is non-negative.
  int num_lanes() const { return do_num_lanes(); }

  /// Returns the Lane indexed by @p index.
  ///
  /// The indexing order is meaningful; numerically adjacent indices correspond
  /// to geometrically adjacent Lanes.  Indices increase "to the left", i.e.,
  /// in the direction of increasing `r` coordinate.
  ///
  /// @pre @p index must be >= 0 and < num_lanes().
  const Lane* lane(int index) const { return do_lane(index); }

 protected:
  Segment() = default;

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the
  /// corresponding public methods.
  ///@{
  virtual SegmentId do_id() const = 0;

  virtual const Junction* do_junction() const = 0;

  virtual int do_num_lanes() const = 0;

  virtual const Lane* do_lane(int index) const = 0;
  ///@}
};

}  // namespace api
}  // namespace maliput
