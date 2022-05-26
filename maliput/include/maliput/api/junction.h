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

class RoadGeometry;
class Segment;

/// Persistent identifier for a Junction element.
using JunctionId = TypeSpecificIdentifier<class Junction>;

/// A Junction is a closed set of Segments which have physically
/// coplanar road surfaces, in the sense that RoadPositions with the
/// same h value (height above surface) in the domains of two Segments
/// map to the same InertialPosition.  The Segments need not be directly
/// connected to one another in the network topology.
///
/// Junctions are grouped by RoadGeometry.
class Junction {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Junction)

  virtual ~Junction() = default;

  /// Returns the persistent identifier.
  JunctionId id() const { return do_id(); }

  /// Returns the RoadGeometry to which this Junction belongs.
  const RoadGeometry* road_geometry() const { return do_road_geometry(); }

  /// Returns the number of Segments in the Junction.
  ///
  /// Return value is non-negative.
  int num_segments() const { return do_num_segments(); }

  /// Returns the Segment indexed by @p index.
  ///
  /// @pre @p index must be >= 0 and < num_segments().
  const Segment* segment(int index) const { return do_segment(index); }

 protected:
  Junction() = default;

 private:
  /// @name NVI implementations of the public methods.
  /// These must satisfy the constraint/invariants of the
  /// corresponding public methods.
  ///@{
  virtual JunctionId do_id() const = 0;

  virtual const RoadGeometry* do_road_geometry() const = 0;

  virtual int do_num_segments() const = 0;

  virtual const Segment* do_segment(int index) const = 0;
  ///@}
};

}  // namespace api
}  // namespace maliput
