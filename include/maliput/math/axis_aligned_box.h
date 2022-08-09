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

#include <optional>
#include <vector>

#include "maliput/common/maliput_copyable.h"
#include "maliput/math/bounding_region.h"
#include "maliput/math/overlapping_type.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace math {

/// Implements BoundingRegion abstract class for axis-aligned-box-shaped bounding regions.
class AxisAlignedBox : public BoundingRegion<Vector3> {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AxisAlignedBox)

  AxisAlignedBox(const Vector3& min_corner, const Vector3& max_corner);
  AxisAlignedBox(const Vector3& min_corner, const Vector3& max_corner, double tolerance);

  ~AxisAlignedBox() = default;

  /// Get the minimum corner of the bounding box.
  const Vector3& min_corner() const { return min_corner_; }
  /// Get the maximum corner of the bounding box.
  const Vector3& max_corner() const { return max_corner_; }

  /// @returns The vertices of the bounding box in the Inertial-frame.
  std::vector<Vector3> get_vertices() const;

  /// @returns True when this region contains @p other .
  bool IsBoxContained(const AxisAlignedBox& other) const;

  /// @returns True when this region intersects @p other .
  bool IsBoxIntersected(const AxisAlignedBox& other) const;

  /// Computes the intersection of this region with @p other .
  /// @param other The other region to intersect with.
  /// @returns The intersection of this region with @p other .
  std::optional<AxisAlignedBox> GetIntersection(const AxisAlignedBox& other) const;

 private:
  static constexpr double kTolerance{1e-14};
  /// Implements BoundingRegion::do_position() method.
  /// @returns Position of the box.
  const Vector3& do_position() const override;

  /// Implements BoundingRegion::DoContains() method.
  /// @param position Inertial-frame's coordinate.
  /// @returns True when @p position is contained in this bounding region.
  bool DoContains(const Vector3& position) const override;

  /// Implements BoundingRegion::DoOverlaps() method.
  /// Valid @p other 's implementations:
  ///  - AxisAlignedBox
  /// @param other Another bounding region.
  /// @returns The overlapping type.
  OverlappingType DoOverlaps(const BoundingRegion<Vector3>& other) const override;

  Vector3 position_;
  Vector3 min_corner_;
  Vector3 max_corner_;
  double tolerance_{};
};

}  // namespace math
}  // namespace maliput
