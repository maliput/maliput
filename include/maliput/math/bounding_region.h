// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet.
// All rights reserved.
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
#include "maliput/math/overlapping_type.h"

namespace maliput {
namespace math {

/// Abstract API for bounding description.
/// @tparam Coordinate Coordinate in a given coordinate system.
template <typename Coordinate>
class BoundingRegion {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BoundingRegion)

  virtual ~BoundingRegion() = default;

  /// Obtains the bounding region's position in the Inertial-frame.
  /// The position is expected to match the centroid of the bounding region.
  /// @returns The position coordinate.
  const Coordinate& position() const { return do_position(); }

  /// Determines whether a @p position in the Inertial-frame is contained in this bounding region.
  /// @param position Inertial-frame's coordinate.
  /// @returns True when @p position is contained in this bounding region.
  bool Contains(const Coordinate& position) const { return DoContains(position); }

  /// Determines the overlapping type with @p other BoundingRegion instance.
  /// - OverlappingType::kDisjointed is returned when there is no overlapping with @p other .
  /// - OverlappingType::kIntersected is returned when @p other intersects with this region.
  /// - OverlappingType::kContained is returned when @p other is contained within this region.
  /// @param other Another BoundingRegion.
  /// @returns The overlapping type.
  OverlappingType Overlaps(const BoundingRegion<Coordinate>& other) const { return DoOverlaps(other); }

 protected:
  BoundingRegion() = default;

 private:
  virtual const Coordinate& do_position() const = 0;
  virtual bool DoContains(const Coordinate& position) const = 0;
  virtual OverlappingType DoOverlaps(const BoundingRegion<Coordinate>& other) const = 0;
};

}  // namespace math
}  // namespace maliput
