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
#include <memory>
#include <limits>
#include <utility>

#include "maliput/common/maliput_copyable.h"
#include "maliput/common/logger.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/math/axis_aligned_box.h"
#include "maliput/math/overlapping_type.h"

namespace maliput {
namespace math {

namespace details {

// Makes a balanced kd-tree from a range of points already loaded in a collection.
// This method is called recursively for building the subtrees.
//
// The @p nodes will be configured via their API for representing a kd-tree.
//
// @tparam Dimension Dimensions of the tree.
// @tparam Node A node in a tree. It must have the following methods:
//   - get_coordinate(): For getting the underlying point.
//   - set_left(Node*): For setting the left sub-node.
//   - set_right(Node*): For setting the right sub-node.
// @tparam NodeCmp A functor for comparing two nodes at certain index/dimension:
//   - NodeCmp::NodeCmp(int index) Constructor.
//   - NodeCmp::operator()(Node* a, Node* b) Comparison operator.
//
// @param begin Is the start of range.
// @param end Is the end of range.
// @param index Is the dimension being evaluated.
// @param nodes Is a list of non-connected nodes to be sorted and configured.
// @returns A pointer to the root node of the tree.
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
  if(left != nullptr) left->set_parent(&nodes[node_index]);
  if(right != nullptr) right->set_parent(&nodes[node_index]);
  nodes[node_index].set_left(left);
  nodes[node_index].set_right(right);
  return &nodes[node_index];
}

// The region corresponding to the left child of a node v at even depth can be computed
// from region(v) as follows :
// region(lc(v)) = region(v)T AND l(v)^left
// where l(v) is the splitting line stored at v, and l(v)^left
// is the half plane to the left of and including l(v).
template <typename Node>
void Initialize3dRegions(bool left, Node* node) {
  MALIPUT_THROW_UNLESS(node->get_parent() != nullptr);
  const auto parent_region = dynamic_cast<const AxisAlignedBox&>(node->get_parent()->get_region());
  Vector3 min_corner{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  Vector3 max_corner{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  const int index = node->get_parent()->get_index();
  if(left){
    max_corner[index] = node->get_parent()->get_coordinate()[index];
  } else {
    min_corner[index] = node->get_parent()->get_coordinate()[index];
  }
  const AxisAlignedBox half_space{min_corner, max_corner};
  const auto region = parent_region.GetIntersection(half_space);
  MALIPUT_THROW_UNLESS(region.has_value());
  node->set_region(std::make_unique<AxisAlignedBox>(region.value()));

  if(node->get_left() != nullptr)
    Initialize3dRegions(true, node->get_left());
  if(node->get_right() != nullptr)
    Initialize3dRegions(false, node->get_right());
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

}  // namespace details

/// KDTree provides a space-partitioning data structure for organizing points in a k-dimensional space.
/// The tree is built from a set of points, where each point is a vector of length k.
/// The tree is built balanced, to guarantee an average of O(log(n)) time for nearest-neighbor queries.
///
/// Inspired on https://rosettacode.org/wiki/K-d_tree.
///
/// @tparam Coordinate Data type being used, must have:
/// - operator[] for accessing the value in each dimension.
/// @tparam Dimension Dimension of the KD-tree.
/// @tparam Distance A functor used for getting the distance between two coordinates. By default,
/// details::SquaredDistance is used.
/// @tparam NodeCmp A functor used for comparing two nodes at certain index/dimension. By default, details::NodeCmp is
/// used.
template <typename CRTP, typename Coordinate, std::size_t Dimension,
          typename Distance = details::SquaredDistance<Coordinate, Dimension>,
          typename NodeCmp = details::NodeCmp<Dimension>>
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
    root_ = details::MakeKdTree<Dimension, Node, NodeCmp>(0, nodes_.size(), 0, nodes_);
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
    root_ = details::MakeKdTree<Dimension, Node, NodeCmp>(0, nodes_.size(), 0, nodes_);
  }

  /// Finds the nearest point in the tree to the given point. (Nearest Neighbour (NN))
  /// Tolerance being used is std::numeric_limits<double>::min().
  /// It is not valid to call this function if the tree is empty.
  /// @param point a point.
  /// @return the nearest point in the tree to the given point
  const Coordinate& Nearest(const Coordinate& point) const {
    return Nearest(point, std::numeric_limits<double>::min());
  }

  /// Finds the nearest point in the tree to the given point. (Nearest Neighbour (NN))
  /// It is not valid to call this function if the tree is empty.
  /// @param point a point.
  /// @param tolerance the maximum distance to the nearest neighbour to be considered a match.
  /// @return the nearest point in the tree to the given point
  /// @throws maliput::common::assertion_error When tree is empty.
  /// @throws maliput::common::assertion_error When tolerance is negative.
  const Coordinate& Nearest(const Coordinate& point, double tolerance) const {
    MALIPUT_VALIDATE(root_ != nullptr, "Tree is empty.");
    MALIPUT_VALIDATE(tolerance > 0, "Tolerance is negative.");
    Node* best = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();
    Nearest(root_, point, 0, tolerance, best, &best_dist);
    return best->get_coordinate();
  }

 protected:
  // A node in the kd-tree.
  // The node is in essence a point of the data structure that divides the upper parent node into two sub-trees, left
  // and right.
  class Node {
   public:
    // Constructs a Node.
    // @param point The point that the node represents.
    Node(const Coordinate& point) : point_(point) {}

    // Returns the point that the node represents.
    const Coordinate& get_coordinate() const { return point_; }

    // Sets @p left as the left sub-node.
    void set_left(Node* left) { left_ = left; }
    // Sets @p right as the right sub-node.
    void set_right(Node* right) { right_ = right; }
    // Sets @p region as the region of the node.
    void set_region(std::unique_ptr<BoundingRegion<Coordinate>> region) { region_ = std::move(region); }
    void set_parent(Node* parent) { parent_ = parent; }
    void set_index(std::size_t index) { index_ = index; }
    // @returns The left sub-node.
    Node* get_left() { return left_; }
    // @returns The right sub-node.
    Node* get_right() { return right_; }
    // @returns The left sub-node.
    Node const* get_left() const { return left_; }
    // @returns The right sub-node.
    Node const* get_right() const { return right_; }
    const Node* get_parent() const { return parent_; }

    // @returns The region of the node.
    const BoundingRegion<Coordinate>& get_region() const { return *region_; }
    std::size_t get_index() const { return index_; }

   private:
    Coordinate point_;
    std::size_t index_{0};
    Node* parent_{nullptr};
    Node* left_{nullptr};
    Node* right_{nullptr};
    std::unique_ptr<BoundingRegion<Coordinate>> region_;
  };

  // Functor for comparing points according to the given dimension being evaluated at that point.

  // Obtains the nearest point in the @p node to the given @p point.
  // @param node The node to be evaluated.
  // @param point The point to be evaluated.
  // @param index Dimension under evaluation as this method is called recursively.
  // @param nearest_neighbour_node The nearest neighbour node so far.
  // @param nearest_neighbour_distance The closest distance to the nearest neighbour so far.
  void Nearest(const Node* node, const Coordinate& point, std::size_t index, double tolerance,
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
    Nearest(dx > 0 ? node->get_left() : node->get_right(), point, index, tolerance, nearest_neighbour_node,
            nearest_neighbour_distance);
    // When going up in the tree, evaluate if the other's node's quadrant is any closer than the current best.
    if (dx * dx >= *nearest_neighbour_distance) return;
    // If the discarded quadrant is closer, evaluate its points.
    Nearest(dx > 0 ? node->get_right() : node->get_left(), point, index, tolerance, nearest_neighbour_node,
            nearest_neighbour_distance);
  }

  // Root node of the tree.
  Node* root_ = nullptr;
  // Nodes in the tree.
  std::deque<Node> nodes_;
};

template <typename Coordinate, ::std::size_t Dimension,
          typename Distance = details::SquaredDistance<Coordinate, Dimension>,
          typename NodeCmp = details::NodeCmp<Dimension>>
class KDTree
    : public KDTreeBase<KDTree<Coordinate, Dimension, Distance, NodeCmp>, Coordinate, Dimension, Distance, NodeCmp> {
 public:
  template <typename Iterator>
  KDTree(Iterator begin, Iterator end)
      : KDTreeBase<KDTree<Coordinate, Dimension, Distance, NodeCmp>, Coordinate, Dimension, Distance, NodeCmp>(begin,
                                                                                                               end) {}

  /// Constructs a KDTreeBase taking a vector of points.
  ///
  /// @param points Vector of points
  /// @tparam Collection type of the collection.
  /// @throws maliput::common::assertion_error When the range is empty.
  template <typename Collection>
  KDTree(Collection&& points)
      : KDTreeBase<KDTree<Coordinate, Dimension, Distance, NodeCmp>, Coordinate, Dimension, Distance, NodeCmp>(points) {
  }
};

template <typename Coordinate, typename Distance = details::SquaredDistance<Coordinate, 3>,
          typename NodeCmp = details::NodeCmp<3>>
class KDTree3D : public KDTreeBase<KDTree3D<Coordinate, Distance, NodeCmp>, Coordinate, 3, Distance, NodeCmp> {
 public:
  template <typename Iterator>
  KDTree3D(Iterator begin, Iterator end)
      : KDTreeBase<KDTree3D<Coordinate, Distance, NodeCmp>, Coordinate, 3, Distance, NodeCmp>(begin, end) {}

  /// Constructs a KDTreeBase taking a vector of points.
  ///
  /// @param points Vector of points
  /// @tparam Collection type of the collection.
  /// @throws maliput::common::assertion_error When the range is empty.
  template <typename Collection>
  KDTree3D(Collection&& points)
      : KDTreeBase<KDTree3D<Coordinate, Distance, NodeCmp>, Coordinate, 3, Distance, NodeCmp>(points) {}

  std::deque<const Coordinate*> RangeSearch(const AxisAlignedBox& region) const {
    std::deque<const Coordinate*> result;
    RangeSearch(this->root_, region, result);
    return result;
  }

  void InitializeRegions() {
    this->root_->set_region(std::make_unique<AxisAlignedBox>(
      Vector3{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()},
      Vector3{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}));
    if(this->root_->get_left() != nullptr)details::Initialize3dRegions(true, this->root_->get_left());
    if(this->root_->get_right() != nullptr)details::Initialize3dRegions(false, this->root_->get_right());
    initialized_regions = true;
  }

 private:
  using KdTreeBaseNode = typename KDTreeBase<KDTree3D<Coordinate, Distance, NodeCmp>, Coordinate, 3, Distance, NodeCmp>::Node;


    // 1: if v is a leaf
    // 2: then Report the stored at v if it lies in R
    // 3: else if region(lv(c)) is fully contained in R
    // 4:   then REPORTSUBTREE(lc(v))
    // 5: else if region(lc(v)) intersects R
    // 6:   then SEARCHKDTREE(lc(v),R)

    // 7: if region(rv(c)) is fully contained in R
    // 8:   then REPORTSUBTREE(rc(v))
    // 9: else if region(lc(v)) intersects R
    // 10:  then SEARCHKDTREE(lc(v),R)
  void RangeSearch(KdTreeBaseNode* node, const AxisAlignedBox& region, std::deque<const Coordinate*>& result) const {
    if(!initialized_regions) {
      maliput::log()->error("Regions weren't initialized. For using KDTree3D::RangeSearch method, you need to call KDTree3D::InitializeRegions() first.");
      return;
    }
    if (node == nullptr) return;
    // If the node is a leaf, check if its coordinate is in the region.
    if (node->get_left() == nullptr && node->get_right() == nullptr) {
      const auto coordinate = node->get_coordinate();
      if (region.Contains(coordinate)) {
        std::cout << coordinate << std::endl;
        result.push_back(&coordinate);
      }
      return;
    }
    // If the region is fully contained in the node, report the node and its children.
    const AxisAlignedBox node_region = dynamic_cast<const AxisAlignedBox&>(node->get_region());
    const auto overlapping_type = region.Overlaps(node_region);
    if(overlapping_type == OverlappingType::kContained) {
      std::cout << "This region is contained in the search region  " << node_region << std::endl;
      ReportPoints(node, result);
      return;
    }
    if(overlapping_type == OverlappingType::kIntersected) {
      RangeSearch(node->get_left(), region, result);
      RangeSearch(node->get_right(), region, result);
      return;
    }
    if(overlapping_type == OverlappingType::kDisjointed) {
      return;
    }
  }

  void ReportPoints(KdTreeBaseNode* node, std::deque<const Coordinate*>& result) const {
    // TODO: Each node could have a list of points located in the region so we don't have to iterate all the nodes for getting the nodes that are contained.
    if (node == nullptr) return;
    if (node->get_left() == nullptr && node->get_right() == nullptr) {
      result.push_back(&(node->get_coordinate()));
      std::cout << "Reported: "<<  node->get_coordinate()<< std::endl;
      return;
    }
    ReportPoints(node->get_left(), result);
    ReportPoints(node->get_right(), result);
  }

  bool initialized_regions = false;
};

}  // namespace math
}  // namespace maliput
