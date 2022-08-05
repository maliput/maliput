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

#include <vector>

#include "maliput/common/maliput_copyable.h"
#include "maliput/math/bounding_region.h"
#include "maliput/math/overlapping_type.h"
#include "maliput/math/roll_pitch_yaw.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace math {

/// Implements BoundingRegion abstract class for non-axis-aligned-box-shaped bounding regions.
class BoundingBox : public BoundingRegion<Vector3> {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BoundingBox)

  /// Constructs a BoundingBox object.
  /// The box is defined by a position, dimensions(length, width and height) and orientation.
  /// @param position Position of the bounding box in the Inertial-frame. The position matches with the centroid of the
  /// box.
  /// @param box_size The size of the bounding box on XYZ (length/width,height)
  /// @param orientation Orientation of the box in the Inertial-frame.
  /// @param tolerance Used to compute IsBoxContained() and IsBoxIntersected() against other BoundingBoxes.
  /// @throws maliput::common::assertion_error When tolerance or any box_size's component are negative.
  BoundingBox(const Vector3& position, const Vector3& box_size, const RollPitchYaw& orientation, double tolerance);

  ~BoundingBox() = default;

  /// @returns The vertices of the bounding box in the Inertial-frame.
  std::vector<Vector3> get_vertices() const;

  /// @returns The orientation of the box in the Inertial-frame.
  const RollPitchYaw& get_orientation() const;

  /// @returns The size of the box in length, width and height.
  const Vector3& box_size() const;

  /// @returns True when this region contains @p other .
  bool IsBoxContained(const BoundingBox& other) const;

  /// @returns True when this region intersects @p other .
  bool IsBoxIntersected(const BoundingBox& other) const;

 private:
  /// Implements BoundingRegion::do_position() method.
  /// @returns Position of the box.
  const Vector3& do_position() const override;

  /// Implements BoundingRegion::DoContains() method.
  /// @param position Inertial-frame's coordinate.
  /// @returns True when @p position is contained in this bounding region.
  bool DoContains(const Vector3& position) const override;

  /// Implements BoundingRegion::DoOverlaps() method.
  /// Valid @p other 's implementations:
  ///  - BoundingBox
  /// @param other Another bounding region.
  /// @returns The overlapping type.
  OverlappingType DoOverlaps(const BoundingRegion<Vector3>& other) const override;

  Vector3 position_;
  Vector3 box_size_;
  RollPitchYaw orientation_;
  double tolerance_{};

  // Half sized box dimensions.
  Vector3 xyz_2_;
};

}  // namespace math
}  // namespace maliput
