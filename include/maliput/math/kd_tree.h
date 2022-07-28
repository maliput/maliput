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

#include <limits>
#include <vector>

#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace math {

/// KDTree provides a space-partitioning data structure for organizing points in a k-dimensional space.
/// The tree is built from a set of points, where each point is a vector of length k.
/// The tree is built balanced, to guarantee an average of O(log(n)) time for nearest-neighbor queries.
///
/// Inspired on https://rosettacode.org/wiki/K-d_tree.
///
/// @tparam Coordinate Data type being used, must have:
/// - operator[] for accessing the value in each dimension.
/// @tparam Dimension Dimension of the KD-tree.
template <typename Coordinate, std::size_t Dimension>
class KDTree {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(KDTree)

  /// Constructs a KDTree taking a pair of iterators. Adds each
  /// point in the range [begin, end) to the tree.
  ///
  /// @param begin start of range
  /// @param end end of range
  /// @tparam Iterator type of the iterator.
  /// @throws maliput::common::assertion_error When the range is empty.
  template <typename Iterator>
  KDTree(Iterator begin, Iterator end);

  /// Constructs a KDTree taking a vector of points.
  ///
  /// @param points Vector of points
  /// @throws maliput::common::assertion_error When the range is empty.
  KDTree(const std::vector<Coordinate>& points);

  /// Returns the number of nodes visited by the last call
  /// to nearest().
  size_t Visited() const { return visited_; }

  /// Returns the distance between the input point and return value
  /// from the last call to nearest().
  double Distance() const;

  /// Finds the nearest point in the tree to the given point. (Nearest Neighbour (NN))
  /// It is not valid to call this function if the tree is empty.
  /// @param point a point
  /// @return the nearest point in the tree to the given point
  const Coordinate& Nearest(const Coordinate& point) const;

 private:
  // Obtains squared distance between two points.
  // @param point_a First point.
  // @param point_b Second point.
  // @returns The squared distance between the two points.
  static double SquaredDistance(const Coordinate& point_a, const Coordinate& point_b);

  // A node in the kd-tree.
  // The node is in essence a point of the data structure that divides the upper parent node into two sub-trees, left
  // and right.
  class Node {
   public:
    // Constructs a Node.
    // @param point The point that the node represents.
    Node(const Coordinate& point) : point_(point) {}

    // Returns the point that the node represents.
    const Coordinate& get_point() const { return point_; }

    // Obtains dimension's value.
    // @param index Dimension to evaluate.
    // @returns The value of the node in the @p index dimension.
    //
    // @throws maliput::common::assertion_error when @p index is greater than Dimension.
    double get(std::size_t index) const;

    // @returns The squared distance between the node and @p other point.
    double Distance(const Coordinate& other) const { return SquaredDistance(point_, other); }

    // Sets @p left as the left sub-node.
    void set_left(Node* left) { left_ = left; }
    // Sets @p right as the right sub-node.
    void set_right(Node* right) { right_ = right; }
    // @returns The left sub-node.
    const Node* get_left() const { return left_; }
    // @returns The right sub-node.
    const Node* get_right() const { return right_; }

   private:
    Coordinate point_;
    Node* left_{nullptr};
    Node* right_{nullptr};
  };

  // Functor for comparing points according to the given dimension being evaluated at that point.
  struct NodeCmp {
    NodeCmp(std::size_t index) : index_(index) {
      MALIPUT_VALIDATE(index < Dimension, "Index can not be greater than number of dimensions minus one.");
    }
    // Compares two nodes according to the given dimension being evaluated at that point.
    bool operator()(const Node& n1, const Node& n2) const { return n1.get(index_) < n2.get(index_); }
    std::size_t index_;
  };

  // Makes a tree from a range of points in the data structure.
  // @param begin Is the start of range.
  // @param end Is the end of range.
  // @param index Is the dimension being evaluated.
  Node* MakeTree(std::size_t begin, std::size_t end, std::size_t index);

  // Obtains the nearest point in the @p node to the given @p point.
  // The @p index under evaluation is provided as this method is called recursively.
  // Updates the #visited, #best_, and #best_dist_ variables.
  void Nearest(const Node* node, const Coordinate& point, std::size_t index) const;

  // TODO(francocipollone): Add these NN-related variables to a struct.
  //
  // Best node found for nearest neighbour(NN) query.
  mutable Node* best_ = nullptr;
  // Best distance found for NN query.
  mutable double best_dist_ = std::numeric_limits<double>::infinity();
  // Number of nodes visited by the last call to Nearest().
  mutable std::size_t visited_ = 0;

  // Root node of the tree.
  Node* root_ = nullptr;
  // Nodes in the tree.
  std::vector<Node> nodes_;
};

}  // namespace math
}  // namespace maliput
