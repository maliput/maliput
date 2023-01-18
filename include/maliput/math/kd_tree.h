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

#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <utility>

#include "maliput/common/logger.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/math/axis_aligned_box.h"
#include "maliput/math/overlapping_type.h"

namespace maliput {
namespace math {
namespace details {

/// Represents a node in a kd-tree data structure.
/// The node is in essence a point of the data structure that divides the upper parent node into two sub-trees, left
/// and right.
/// @tparam Coordinate The type of the coordinate.
/// @tparam Region The type of the region. For example for 3D space, the region could be maliput::math::AxisAlignedBox.
template <typename Coordinate, typename Region>
class Node {
 public:
  /// Constructs a Node.
  /// @param point The point that the node represents.
  Node(const Coordinate& point) : point_(point) {}

  /// Returns the point that the node represents.
  const Coordinate& get_coordinate() const { return point_; }

  /// Sets @p left as the left sub-node.
  void set_left(Node* left) { left_ = left; }
  /// Sets @p right as the right sub-node.
  void set_right(Node* right) { right_ = right; }
  /// Sets @p parent as the parent of the node.
  void set_parent(Node* parent) { parent_ = parent; }
  /// Sets @p region as the region of the node.
  void set_region(std::unique_ptr<Region> region) { region_ = std::move(region); }
  /// Stores the dimension being evaluated for the kd-tree algorithm.
  void set_index(std::size_t index) { index_ = index; }

  /// @returns The left sub-node.
  Node* get_left() { return left_; }
  /// @returns The right sub-node.
  Node* get_right() { return right_; }
  /// @returns The left sub-node.
  Node const* get_left() const { return left_; }
  /// @returns The right sub-node.
  Node const* get_right() const { return right_; }
  /// @returns The parent of the node.
  const Node* get_parent() const { return parent_; }
  /// @returns The region of the node.
  const Region& get_region() const { return *region_; }
  /// @returns The dimension being evaluated for the kd-tree algorithm.
  std::size_t get_index() const { return index_; }

 private:
  Coordinate point_;
  std::size_t index_{0};
  Node* parent_{nullptr};
  Node* left_{nullptr};
  Node* right_{nullptr};
  std::unique_ptr<Region> region_;
};

/// Makes a balanced kd-tree from a range of points already loaded in a collection.
/// This method is called recursively for building the subtrees.
///
/// The @p nodes will be configured via their API for representing a kd-tree.
///
/// @tparam Dimension Dimensions of the tree.
/// @tparam Node A node in a tree. It must have the following methods:
///   - get_coordinate(): For getting the underlying point.
///   - set_left(Node*): For setting the left sub-node.
///   - set_right(Node*): For setting the right sub-node.
/// @tparam NodeCmp A functor for comparing two nodes at certain index/dimension:
///   - NodeCmp::NodeCmp(int index) Constructor.
///   - NodeCmp::operator()(Node* a, Node* b) Comparison operator.
///
/// @param begin Is the start of range.
/// @param end Is the end of range.
/// @param index Is the dimension being evaluated.
/// @param nodes Is a list of non-connected nodes to be sorted and configured.
/// @returns A pointer to the root node of the tree.
template <std::size_t Dimension, typename Node, typename NodeCmp>
Node* MakeKdTree(std::size_t begin, std::size_t end, std::size_t index, std::deque<Node>& nodes) {
  static_assert(Dimension > 0, "Dimension must be greater than 0.");
  // If range is empty, no tree is needed to be built.
  if (end <= begin) return nullptr;
  const std::size_t node_index = begin + (end - begin) / 2;
  const auto i = nodes.begin();
  // Sorting the element in the middle of the range(median).
  // Smaller and greater values will be located to the left and right of the range correspondingly, according to
  // NodeCmp functor. However, those values aren't sorted.
  std::nth_element(i + begin, i + node_index, i + end, NodeCmp(index));
  // Storing the index used in this dimension.
  nodes[node_index].set_index(index);
  // Obtaining the index to be used for sorting in the next call to MakeTree.
  index = (index + 1) % Dimension;
  Node* left = MakeKdTree<Dimension, Node, NodeCmp>(begin, node_index, index, nodes);
  Node* right = MakeKdTree<Dimension, Node, NodeCmp>(node_index + 1, end, index, nodes);
  if (left != nullptr) left->set_parent(&nodes[node_index]);
  if (right != nullptr) right->set_parent(&nodes[node_index]);
  nodes[node_index].set_left(left);
  nodes[node_index].set_right(right);
  return &nodes[node_index];
}

/// Computes the regions corresponding to each node in a tree and stores them in the nodes.
///
/// The region corresponding to the left child(`lc`) of a node `v` at even depth can be computed
/// as follows:
///
///     region(lc(v)) = region(v) ∩ sp(v)^left
///
/// where `sp(v)` is the splitting plane stored at `v`, and `sp(v)^left`
/// is the half-space to the left of and including `sp(v)`.
/// For the right child the computation is analogous.
/// @tparam Node A node in a tree. It must have the following methods:
///         - get_coordinate(): For getting the underlying point.
///         - get_parent(): For getting the parent of the node.
///         - get_left(): For getting the left sub-node.
///         - get_right(): For getting the right sub-node.
/// @param left True if the node is the left child of its parent.
/// @param node The node to compute the region for.
template <typename Node>
void Initialize3dRegions(bool left, Node* node) {
  MALIPUT_THROW_UNLESS(node != nullptr);
  MALIPUT_THROW_UNLESS(node->get_parent() != nullptr);
  const auto parent_region = node->get_parent()->get_region();
  const double infinity = std::numeric_limits<double>::infinity();
  Vector3 min_corner{-infinity, -infinity, -infinity};
  Vector3 max_corner{infinity, infinity, infinity};
  const int index = node->get_parent()->get_index();
  if (left) {
    max_corner[index] = node->get_parent()->get_coordinate()[index];
  } else {
    min_corner[index] = node->get_parent()->get_coordinate()[index];
  }
  const AxisAlignedBox half_space{min_corner, max_corner};
  const auto region = parent_region.GetIntersection(half_space);
  MALIPUT_THROW_UNLESS(region.has_value());
  node->set_region(std::make_unique<AxisAlignedBox>(region.value()));

  if (node->get_left() != nullptr) Initialize3dRegions(true, node->get_left());
  if (node->get_right() != nullptr) Initialize3dRegions(false, node->get_right());
}

/// Calculates the squared distance between two points.
/// @tparam Coordinate The type of the coordinates.
/// @tparam Dimension The dimension of the points.
template <typename Coordinate, std::size_t Dimension>
struct SquaredDistance {
  static_assert(Dimension > 0, "Dimension must be greater than 0.");

  /// Obtains squared distance between two coordinates.
  /// @param lhs First point.
  /// @param rhs Second point.
  /// @returns The squared distance between the two coordinates.
  double operator()(const Coordinate& lhs, const Coordinate& rhs) const {
    double dist = 0;
    for (std::size_t i = 0; i < Dimension; ++i) {
      const double d = lhs[i] - rhs[i];
      dist += d * d;
    }
    return dist;
  }
};

/// Functor for comparing points according to the given dimension being evaluated at that point.
/// @tparam Dimension The dimension of the points.
template <std::size_t Dimension>
struct NodeCmp {
  NodeCmp(std::size_t index) : index_(index) {
    static_assert(Dimension > 0, "Dimension must be greater than 0.");
    MALIPUT_VALIDATE(index < Dimension, "Index can not be greater than number of dimensions minus one.");
  }
  /// Compares two nodes according to the given dimension being evaluated at that point.
  /// @tparam Node The type of the nodes.
  ///         It must provide a method get_coordinate() for getting the underlying point.
  /// @param lhs First node.
  /// @param rhs Second node.
  template <typename Node>
  bool operator()(const Node& lhs, const Node& rhs) const {
    return lhs.get_coordinate()[index_] < rhs.get_coordinate()[index_];
  }

  const std::size_t index_{};
};

/// KDTree provides a space-partitioning data structure for organizing points in a k-dimensional space.
/// The tree is built from a set of points, where each point is a vector of length k.
/// The tree is built balanced, to guarantee an average of O(log(n)) time for nearest-neighbor queries.
///
/// Inspired on https://rosettacode.org/wiki/K-d_tree.
///
/// @tparam CRTP Derived class. It follows the curiously recurring template pattern.
/// @tparam Coordinate Data type being used, must have:
/// - operator[] for accessing the value in each dimension.
/// @tparam Dimension Dimension of the KD-tree.
/// @tparam Region The type of the region.
/// @tparam Distance A functor used for getting the distance between two coordinates. By default,
/// details::SquaredDistance is used.
/// @tparam NodeCmp A functor used for comparing two nodes at certain index/dimension. By default, details::NodeCmp is
/// used.
template <typename CRTP, typename Coordinate, std::size_t Dimension, typename Region = BoundingRegion<Coordinate>,
          typename Distance = SquaredDistance<Coordinate, Dimension>, typename NodeCmp = NodeCmp<Dimension>>
class KDTreeBase {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(KDTreeBase)
  static_assert(Dimension > 0, "Dimension must be greater than 0.");

  /// Constructs a KDTreeBase taking a pair of iterators. Adds each
  /// point in the range [begin, end) to the tree.
  ///
  /// @param begin start of range
  /// @param end end of range
  /// @tparam Iterator type of the iterator.
  /// @throws maliput::common::assertion_error When the range is empty.
  template <typename Iterator>
  KDTreeBase(Iterator begin, Iterator end) : nodes_(begin, end) {
    MALIPUT_VALIDATE(!nodes_.empty(), "Empty range");
    root_ = MakeKdTree<Dimension, Node, NodeCmp>(0, nodes_.size(), 0, nodes_);
  }

  /// Constructs a KDTreeBase taking a vector of points.
  ///
  /// @param points Vector of points
  /// @tparam Collection type of the collection.
  /// @throws maliput::common::assertion_error When the range is empty.
  template <typename Collection>
  KDTreeBase(Collection&& points) {
    MALIPUT_VALIDATE(!points.empty(), "Empty range");
    for (auto&& point : points) {
      nodes_.emplace_back(std::forward<Coordinate>(point));
    }
    root_ = MakeKdTree<Dimension, Node, NodeCmp>(0, nodes_.size(), 0, nodes_);
  }

  /// Finds the nearest point in the tree to the given point. (Nearest Neighbour (NN))
  /// Tolerance being used is std::numeric_limits<double>::min().
  /// It is not valid to call this function if the tree is empty.
  /// @param point a point.
  /// @return the nearest point in the tree to the given point
  const Coordinate& nearest_point(const Coordinate& point) const {
    return nearest_point(point, std::numeric_limits<double>::min());
  }

  /// Finds the nearest point in the tree to the given point. (Nearest Neighbour (NN))
  /// It is not valid to call this function if the tree is empty.
  /// @param point a point.
  /// @param tolerance the maximum distance to the nearest neighbour to be considered a match.
  /// @return the nearest point in the tree to the given point
  /// @throws maliput::common::assertion_error When tree is empty.
  /// @throws maliput::common::assertion_error When tolerance is negative.
  const Coordinate& nearest_point(const Coordinate& point, double tolerance) const {
    MALIPUT_VALIDATE(root_ != nullptr, "Tree is empty.");
    MALIPUT_VALIDATE(tolerance > 0, "Tolerance is negative.");
    Node* best = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();
    nearest_point(root_, point, 0, tolerance, best, &best_dist);
    return best->get_coordinate();
  }

 protected:
  using Node = details::Node<Coordinate, Region>;

  // Functor for comparing points according to the given dimension being evaluated at that point.

  // Obtains the nearest point in the @p node to the given @p point.
  // @param node The node to be evaluated.
  // @param point The point to be evaluated.
  // @param index Dimension under evaluation as this method is called recursively.
  // @param nearest_neighbour_node The nearest neighbour node so far.
  // @param nearest_neighbour_distance The closest distance to the nearest neighbour so far.
  void nearest_point(const Node* node, const Coordinate& point, std::size_t index, double tolerance,
                     Node*& nearest_neighbour_node, double* nearest_neighbour_distance) const {
    MALIPUT_VALIDATE(index < Dimension, "Index can not be greater than number of dimensions minus one.");
    MALIPUT_THROW_UNLESS(nearest_neighbour_distance != nullptr);
    if (node == nullptr) return;
    // Get the distance between the point and the current node and update best result if necessary.
    const double node_point_distance = Distance()(node->get_coordinate(), point);
    if (nearest_neighbour_node == nullptr || node_point_distance < *nearest_neighbour_distance) {
      *nearest_neighbour_distance = node_point_distance;
      nearest_neighbour_node = const_cast<Node*>(node);
    }
    // If the distance is less than tolerance, return
    if (*nearest_neighbour_distance < tolerance) return;
    // Evaluate if moving to right or left node.
    const double dx = node->get_coordinate()[index] - point[index];
    // Compute index value for the next MakeTree call.
    index = (index + 1) % Dimension;
    nearest_point(dx > 0 ? node->get_left() : node->get_right(), point, index, tolerance, nearest_neighbour_node,
                  nearest_neighbour_distance);
    // When going up in the tree, evaluate if the other's node's quadrant is any closer than the current best.
    if (dx * dx >= *nearest_neighbour_distance) return;
    // If the discarded quadrant is closer, evaluate its points.
    nearest_point(dx > 0 ? node->get_right() : node->get_left(), point, index, tolerance, nearest_neighbour_node,
                  nearest_neighbour_distance);
  }

  // Root node of the tree.
  Node* root_ = nullptr;
  // Nodes in the tree.
  std::deque<Node> nodes_;
};

}  // namespace details

/// N-Dimension KDTree.
/// See KDTreeBase for details.
template <typename Coordinate, ::std::size_t Dimension, typename Region = BoundingRegion<Coordinate>,
          typename Distance = details::SquaredDistance<Coordinate, Dimension>,
          typename NodeCmp = details::NodeCmp<Dimension>>
class KDTree : public details::KDTreeBase<KDTree<Coordinate, Dimension, Region, Distance, NodeCmp>, Coordinate,
                                          Dimension, Region, Distance, NodeCmp> {
 public:
  template <typename Iterator>
  KDTree(Iterator begin, Iterator end)
      : details::KDTreeBase<KDTree<Coordinate, Dimension, Region, Distance, NodeCmp>, Coordinate, Dimension, Region,
                            Distance, NodeCmp>(begin, end) {}

  /// Constructs a KDTreeBase taking a vector of points.
  ///
  /// @param points Vector of points
  /// @tparam Collection type of the collection.
  /// @throws maliput::common::assertion_error When the range is empty.
  template <typename Collection>
  KDTree(Collection&& points)
      : details::KDTreeBase<KDTree<Coordinate, Dimension, Region, Distance, NodeCmp>, Coordinate, Dimension, Region,
                            Distance, NodeCmp>(points) {}
};

/// 3-Dimensional KDTree.
/// In addition it provides a RangeSearch method for range queries in the 3D-space.
/// @code {.cpp}
///  KDTree<Vector3> tree{points};
///  tree.RangeSearch(region_1);
///  tree.RangeSearch(region_2);
///  ...
///  tree.nearest_point(point_1); // NearestNeighbour (NN).
/// @endcode

/// @tparam Coordinate Data type being used, must have:
/// - operator[] for accessing the value in each dimension.
/// @tparam Dimension Dimension of the KD-tree.
/// @tparam Distance A functor used for getting the distance between two coordinates. By default,
/// details::SquaredDistance is used.
/// @tparam NodeCmp A functor used for comparing two nodes at certain index/dimension. By default, details::NodeCmp is
/// used.
template <typename Coordinate, typename Distance = details::SquaredDistance<Coordinate, 3>,
          typename NodeCmp = details::NodeCmp<3>>
class KDTree3D : public details::KDTreeBase<KDTree3D<Coordinate, Distance, NodeCmp>, Coordinate, 3, AxisAlignedBox,
                                            Distance, NodeCmp> {
 public:
  template <typename Iterator>
  KDTree3D(Iterator begin, Iterator end)
      : details::KDTreeBase<KDTree3D<Coordinate, Distance, NodeCmp>, Coordinate, 3, AxisAlignedBox, Distance, NodeCmp>(
            begin, end) {
    InitializeRegions();
  }

  /// Constructs a KDTreeBase taking a vector of points.
  ///
  /// @param points Vector of points
  /// @tparam Collection type of the collection.
  /// @throws maliput::common::assertion_error When the range is empty.
  template <typename Collection>
  KDTree3D(Collection&& points)
      : details::KDTreeBase<KDTree3D<Coordinate, Distance, NodeCmp>, Coordinate, 3, AxisAlignedBox, Distance, NodeCmp>(
            points) {
    InitializeRegions();
  }

  /// Range search in the 3D-space.
  /// @param region The region to be searched Coordinates on.
  /// @returns A vector of Coordinates located in the @p region.
  /// For further info on Range Search algorithm see http://www.cs.utah.edu/~lifeifei/cis5930/kdtree.pdf
  std::deque<const Coordinate*> RangeSearch(const AxisAlignedBox& region) const {
    std::deque<const Coordinate*> result;
    RangeSearch(this->root_, region, result);
    return result;
  }

 private:
  using KdTreeBaseNode = typename details::KDTreeBase<KDTree3D<Coordinate, Distance, NodeCmp>, Coordinate, 3,
                                                      AxisAlignedBox, Distance, NodeCmp>::Node;

  /// Initializes the regions of the nodes.
  /// This initialization must be run before calling the RangeSearch method.
  /// See RangeSearch method.
  void InitializeRegions() {
    this->root_->set_region(std::make_unique<AxisAlignedBox>(
        Vector3{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity()},
        Vector3{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()}));
    if (this->root_->get_left() != nullptr) details::Initialize3dRegions(true, this->root_->get_left());
    if (this->root_->get_right() != nullptr) details::Initialize3dRegions(false, this->root_->get_right());
  }

  // Performs a range search query recursively.
  // @pre InitializeRegions() must be called before using this method.
  // @param node The node to be searched.
  // @param region The search region.
  // @param result The result of the search.
  void RangeSearch(KdTreeBaseNode* node, const AxisAlignedBox& region, std::deque<const Coordinate*>& result) const {
    if (node == nullptr) return;
    // If the node is a leaf, check if its coordinate is in the region.
    if (node->get_left() == nullptr && node->get_right() == nullptr) {
      const auto& coordinate = node->get_coordinate();
      if (region.Contains({coordinate[0], coordinate[1], coordinate[2]})) {
        result.push_back(&coordinate);
      }
      return;
    }
    // If the region is fully contained in the node, report the node and its children.
    const auto overlapping_type = region.Overlaps(node->get_region());

    if (overlapping_type == OverlappingType::kContained) {
      ReportPointsInSubTree(node, result);
      return;
    }
    if (overlapping_type == OverlappingType::kIntersected) {
      const auto& coordinate = node->get_coordinate();
      if (region.Contains({coordinate[0], coordinate[1], coordinate[2]})) {
        result.push_back(&coordinate);
      }
      RangeSearch(node->get_left(), region, result);
      RangeSearch(node->get_right(), region, result);
      return;
    }
    if (overlapping_type == OverlappingType::kDisjointed) {
      return;
    }
  }

  // Reports the points in the subtree rooted at @p node.
  // @param node The node to be searched.
  // @param result The result of the search.
  void ReportPointsInSubTree(KdTreeBaseNode* node, std::deque<const Coordinate*>& result) const {
    // TODO(francocipollone): Each node could have a list of references to points located within the region so we don't
    //                        have to iterate all the nodes for getting the nodes that are contained.
    if (node == nullptr) return;
    result.push_back(&(node->get_coordinate()));
    ReportPointsInSubTree(node->get_left(), result);
    ReportPointsInSubTree(node->get_right(), result);
  }
};

}  // namespace math
}  // namespace maliput
