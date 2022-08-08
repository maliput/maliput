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
#include "maliput/math/axis_aligned_box.h"

namespace maliput {
namespace math {
AxisAlignedBox::AxisAlignedBox(const Vector3& min_corner, const Vector3& max_corner)
    : AxisAlignedBox(min_corner, max_corner, kTolerance) {}

AxisAlignedBox::AxisAlignedBox(const Vector3& min_corner, const Vector3& max_corner, double tolerance)
    : min_corner_(min_corner), max_corner_(max_corner), tolerance_(tolerance) {
  MALIPUT_THROW_UNLESS(tolerance >= 0.);
  MALIPUT_THROW_UNLESS(min_corner_.x() <= max_corner_.x());
  MALIPUT_THROW_UNLESS(min_corner_.y() <= max_corner_.y());
  MALIPUT_THROW_UNLESS(min_corner_.z() <= max_corner_.z());
  position_ = 0.5 * min_corner_ + 0.5 * max_corner_;
}

const Vector3& AxisAlignedBox::do_position() const { return position_; }

std::vector<Vector3> AxisAlignedBox::get_vertices() const {
  return {{min_corner_.x(), min_corner_.y(), min_corner_.z()}, {min_corner_.x(), min_corner_.y(), max_corner_.z()},
          {min_corner_.x(), max_corner_.y(), min_corner_.z()}, {min_corner_.x(), max_corner_.y(), max_corner_.z()},
          {max_corner_.x(), min_corner_.y(), min_corner_.z()}, {max_corner_.x(), min_corner_.y(), max_corner_.z()},
          {max_corner_.x(), max_corner_.y(), min_corner_.z()}, {max_corner_.x(), max_corner_.y(), max_corner_.z()}};
}

bool AxisAlignedBox::DoContains(const Vector3& position) const {
  return position.x() >= min_corner_.x() - tolerance_ && position.x() <= max_corner_.x() + tolerance_ &&
         position.y() >= min_corner_.y() - tolerance_ && position.y() <= max_corner_.y() + tolerance_ &&
         position.z() >= min_corner_.z() - tolerance_ && position.z() <= max_corner_.z() + tolerance_;
}

OverlappingType AxisAlignedBox::DoOverlaps(const BoundingRegion<Vector3>& other) const {
  const auto other_aa_box = dynamic_cast<const AxisAlignedBox*>(&other);
  MALIPUT_VALIDATE(other_aa_box != nullptr, "BoundingRegion's implementations supported: AxisAlignedBox.");

  if (IsBoxIntersected(*other_aa_box)) {
    if (IsBoxContained(*other_aa_box)) {
      return OverlappingType::kContained;
    } else {
      return OverlappingType::kIntersected;
    }
  } else {
    return OverlappingType::kDisjointed;
  }
}

bool AxisAlignedBox::IsBoxIntersected(const AxisAlignedBox& other) const {
  // Check the six separating planes.
  return !(max_corner_.x() < other.min_corner_.x() || other.max_corner_.x() < min_corner_.x() ||
           max_corner_.y() < other.min_corner_.y() || other.max_corner_.y() < min_corner_.y() ||
           max_corner_.z() < other.min_corner_.z() || other.max_corner_.z() < min_corner_.z());
}

bool AxisAlignedBox::IsBoxContained(const AxisAlignedBox& other) const {
  const auto vertices = other.get_vertices();
  return std::all_of(vertices.begin(), vertices.end(), [this](const auto& vertex) { return this->DoContains(vertex); });
}

std::optional<AxisAlignedBox> AxisAlignedBox::GetIntersection(const AxisAlignedBox& other) const {
  if (!IsBoxIntersected(other)) {
    return std::nullopt;
  }
  const auto min_corner =
      Vector3(std::max(min_corner_.x(), other.min_corner_.x()), std::max(min_corner_.y(), other.min_corner_.y()),
              std::max(min_corner_.z(), other.min_corner_.z()));
  const auto max_corner =
      Vector3(std::min(max_corner_.x(), other.max_corner_.x()), std::min(max_corner_.y(), other.max_corner_.y()),
              std::min(max_corner_.z(), other.max_corner_.z()));
  return {AxisAlignedBox(min_corner, max_corner, tolerance_)};
}

}  // namespace math
}  // namespace maliput
