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
#include "maliput/math/bounding_box.h"

namespace maliput {
namespace math {

BoundingBox::BoundingBox(const Vector3& position, const Vector3& box_size, const RollPitchYaw& orientation,
                         double tolerance)
    : position_(position),
      box_size_(box_size),
      orientation_(orientation),
      tolerance_(tolerance),
      xyz_2_(box_size.x() / 2., box_size.y() / 2., box_size.z() / 2.) {
  MALIPUT_THROW_UNLESS(tolerance >= 0.);
  MALIPUT_THROW_UNLESS(box_size.x() >= 0.);
  MALIPUT_THROW_UNLESS(box_size.y() >= 0.);
  MALIPUT_THROW_UNLESS(box_size.z() >= 0.);
}

const Vector3& BoundingBox::do_position() const { return position_; }

std::vector<Vector3> BoundingBox::get_vertices() const {
  const std::vector<Vector3> vertices_box_frame{
      {xyz_2_.x(), xyz_2_.y(), xyz_2_.z()},   {-xyz_2_.x(), xyz_2_.y(), xyz_2_.z()},
      {xyz_2_.x(), -xyz_2_.y(), xyz_2_.z()},  {xyz_2_.x(), xyz_2_.y(), -xyz_2_.z()},
      {-xyz_2_.x(), -xyz_2_.y(), xyz_2_.z()}, {xyz_2_.x(), -xyz_2_.y(), -xyz_2_.z()},
      {-xyz_2_.x(), xyz_2_.y(), -xyz_2_.z()}, {-xyz_2_.x(), -xyz_2_.y(), -xyz_2_.z()},
  };
  std::vector<Vector3> vertices;
  for (const auto& vertex : vertices_box_frame) {
    vertices.push_back(orientation_.ToMatrix().inverse() * vertex + position_);
  }
  return vertices;
}

const Vector3& BoundingBox::box_size() const { return box_size_; }

const RollPitchYaw& BoundingBox::get_orientation() const { return orientation_; }

bool BoundingBox::DoContains(const Vector3& position) const {
  const Vector3 box_frame_position = orientation_.ToMatrix() * (position - position_);
  return box_frame_position.x() <= xyz_2_.x() + tolerance_ && box_frame_position.x() >= -xyz_2_.x() - tolerance_ &&
         box_frame_position.y() <= xyz_2_.y() + tolerance_ && box_frame_position.y() >= -xyz_2_.y() - tolerance_ &&
         box_frame_position.z() <= xyz_2_.z() + tolerance_ && box_frame_position.z() >= -xyz_2_.z() - tolerance_;
}

OverlappingType BoundingBox::DoOverlaps(const BoundingRegion<Vector3>& other) const {
  auto other_box = dynamic_cast<const BoundingBox*>(&other);
  MALIPUT_VALIDATE(other_box != nullptr, "BoundingRegion's implementations supported: BoundingBox.");
  if (IsBoxContained(*other_box)) {
    return OverlappingType::kContained;
  }
  if (IsBoxIntersected(*other_box)) {
    return OverlappingType::kIntersected;
  }
  return OverlappingType::kDisjointed;
}

bool BoundingBox::IsBoxContained(const BoundingBox& other) const {
  const auto vertices = other.get_vertices();
  return std::all_of(vertices.begin(), vertices.end(), [this](const auto& vertex) { return this->DoContains(vertex); });
}

bool BoundingBox::IsBoxIntersected(const BoundingBox& other) const {
  // The following is based on Drake's implementation of drake::geometry::internal::BoxesOverlap() method.
  // See https://github.com/RobotLocomotion/drake/blob/master/geometry/proximity/boxes_overlap.cc

  // Let's compute the relative transformation from this box to the other box.
  // For the purposes of streamlining the math below, translation and rotation
  // will be named `t` and `r` respectively.
  const Vector3 t = other.position() - position();
  const Matrix3 r = get_orientation().ToMatrix().inverse() * other.get_orientation().ToMatrix();

  // Compute some common subexpressions and add epsilon to counteract
  // arithmetic error, e.g. when two edges are parallel. We use the value as
  // specified from Gottschalk's OBB robustness tests.
  const double kEpsilon = 0.000001;
  Matrix3 abs_r = r;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      abs_r[i][j] = std::abs(abs_r[i][j]) + kEpsilon;
    }
  }

  // First category of cases separating along a's axes.
  for (int i = 0; i < 3; ++i) {
    // if (std::abs(t[i]) > xyz_2_[i] + other.xyz_2_.dot(abs_r.block<1, 3>(i, 0))) {
    const Vector3 abs_r_i = abs_r.col(i);
    if (std::abs(t[i]) > xyz_2_[i] + other.xyz_2_.dot(abs_r_i)) {
      return false;
    }
  }

  // Second category of cases separating along b's axes.
  for (int i = 0; i < 3; ++i) {
    // if (std::abs(t.dot(r.block<3, 1>(0, i))) >
    // other.xyz_2_[i] + xyz_2_.dot(abs_r.block<3, 1>(0, i))) {
    const Vector3 r_i = r.col(i);
    const Vector3 abs_r_i = abs_r.col(i);
    if (std::abs(t.dot(r_i)) > other.xyz_2_[i] + xyz_2_.dot(abs_r_i)) {
      return false;
    }
  }

  // Third category of cases separating along the axes formed from the cross
  // products of a's and b's axes.
  int i1 = 1;
  for (int i = 0; i < 3; ++i) {
    const int i2 = (i1 + 1) % 3;  // Calculate common sub expressions.
    int j1 = 1;
    for (int j = 0; j < 3; ++j) {
      const int j2 = (j1 + 1) % 3;
      if (std::abs(t[i2] * r[i1][j] - t[i1] * r[i2][j]) > xyz_2_[i1] * abs_r[i2][j] + xyz_2_[i2] * abs_r[i1][j] +
                                                              other.xyz_2_[j1] * abs_r[i][j2] +
                                                              other.xyz_2_[j2] * abs_r[i][j1]) {
        return false;
      }
      j1 = j2;
    }
    i1 = i2;
  }

  return true;
}

}  // namespace math
}  // namespace maliput
