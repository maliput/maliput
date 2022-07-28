// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
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

#include "maliput/math/kd_tree.h"

#include <algorithm>
#include <cmath>

#include "maliput/math/vector.h"

namespace maliput {
namespace math {

template <typename Coordinate, std::size_t Dimension>
template <typename Iterator>
KDTree<Coordinate, Dimension>::KDTree(Iterator begin, Iterator end) : nodes_(begin, end) {
  // TODO: verifies that there is at least one point.
  MALIPUT_VALIDATE(!nodes_.empty(), "Empty range");
  root_ = MakeTree(0, nodes_.size(), 0);
}

template <typename Coordinate, std::size_t Dimension>
KDTree<Coordinate, Dimension>::KDTree(const std::vector<Coordinate>& points) {
  MALIPUT_VALIDATE(!points.empty(), "Empty range");
  for (const auto& point : points) {
    nodes_.push_back(point);
  }
  root_ = MakeTree(0, nodes_.size(), 0);
}

template <typename Coordinate, std::size_t Dimension>
double KDTree<Coordinate, Dimension>::Distance() const {
  return std::sqrt(best_dist_);
}

template <typename Coordinate, std::size_t Dimension>
const Coordinate& KDTree<Coordinate, Dimension>::Nearest(const Coordinate& point) const {
  MALIPUT_VALIDATE(root_ != nullptr, "Tree is empty.");
  best_ = nullptr;
  visited_ = 0;
  best_dist_ = 0;
  Nearest(root_, point, 0);
  return best_->get_point();
}

template <typename Coordinate, std::size_t Dimension>
double KDTree<Coordinate, Dimension>::SquaredDistance(const Coordinate& point_a, const Coordinate& point_b) {
  double dist = 0;
  for (std::size_t i = 0; i < Dimension; ++i) {
    const double d = point_a[i] - point_b[i];
    dist += d * d;
  }
  return dist;
}

template <typename Coordinate, std::size_t Dimension>
KDTree<Coordinate, Dimension>::NodeCmp::NodeCmp(std::size_t index) : index_(index) {
  MALIPUT_VALIDATE(index < Dimension, "Index can not be greater than number of dimensions minus one.");
}

template <typename Coordinate, std::size_t Dimension>
bool KDTree<Coordinate, Dimension>::NodeCmp::operator()(const Node& n1, const Node& n2) const { return n1.get(index_) < n2.get(index_); }


template <typename Coordinate, std::size_t Dimension>
typename KDTree<Coordinate, Dimension>::Node* KDTree<Coordinate, Dimension>::MakeTree(std::size_t begin,
                                                                                      std::size_t end,
                                                                                      std::size_t index) {
  // If range is empty, no tree is needed to be built.
  if (end <= begin) return nullptr;
  const std::size_t n = begin + (end - begin) / 2;
  auto i = nodes_.begin();
  // Sorting the element in the middle of the range(median).
  // Smaller and greater values will be located to the left and right of the range correspondingly, according to
  // NodeCmp functor. However, those values aren't sorted.
  std::nth_element(i + begin, i + n, i + end, NodeCmp(index));
  // Obtaining the index to be used for sorting in the next call to MakeTree.
  index = (index + 1) % Dimension;
  nodes_[n].set_left(MakeTree(begin, n, index));
  nodes_[n].set_right(MakeTree(n + 1, end, index));
  return &nodes_[n];
}

template <typename Coordinate, std::size_t Dimension>
void KDTree<Coordinate, Dimension>::Nearest(const typename KDTree<Coordinate, Dimension>::Node* node,
                                            const Coordinate& point, std::size_t index) const {
  if (node == nullptr) return;
  ++visited_;
  // Get the distance between the point and the current node and update best result if necessary.
  const double node_point_distance = node->Distance(point);
  if (best_ == nullptr || node_point_distance < best_dist_) {
    best_dist_ = node_point_distance;
    best_ = const_cast<Node*>(node);
  }
  // If the distance is less than numeric limit, return
  if (best_dist_ < std::numeric_limits<double>::min()) return;
  // Evaluate if moving to right or left node.
  const double dx = node->get(index) - point[index];
  // Compute index value for the next MakeTree call.
  index = (index + 1) % Dimension;
  Nearest(dx > 0 ? node->get_left() : node->get_right(), point, index);
  // When going up in the tree, evaluate if the other's node's quadrant is any closer than the current best.
  if (dx * dx >= best_dist_) return;
  // If the discarded quadrant is closer, evaluate its points.
  Nearest(dx > 0 ? node->get_right() : node->get_left(), point, index);
}

template <typename Coordinate, std::size_t Dimension>
double KDTree<Coordinate, Dimension>::Node::get(std::size_t index) const {
  MALIPUT_VALIDATE(index < Dimension, "Index can not be greater than number of dimensions minus one.");
  return point_[index];
}

}  // namespace math
}  // namespace maliput

// Explicit instantiations.
template class std::vector<maliput::math::Vector3>;
template class maliput::math::KDTree<maliput::math::Vector3, 3>;
template maliput::math::KDTree<maliput::math::Vector3, 3>::KDTree(std::vector<maliput::math::Vector3>::iterator,
                                                                  std::vector<maliput::math::Vector3>::iterator);
template maliput::math::KDTree<maliput::math::Vector3, 3>::KDTree(std::vector<maliput::math::Vector3>::const_iterator,
                                                                  std::vector<maliput::math::Vector3>::const_iterator);

template class std::vector<std::array<double, 3>>;
template class maliput::math::KDTree<std::array<double, 3>, 3>;
template maliput::math::KDTree<std::array<double, 3>, 3>::KDTree(std::vector<std::array<double, 3>>::iterator,
                                                                 std::vector<std::array<double, 3>>::iterator);
template maliput::math::KDTree<std::array<double, 3>, 3>::KDTree(std::vector<std::array<double, 3>>::const_iterator,
                                                                 std::vector<std::array<double, 3>>::const_iterator);
