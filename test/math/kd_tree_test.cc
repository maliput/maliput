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

#include <random>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace math {
namespace {

// Tests maliput::math::details::SquaredDistance functor.
class SquaredDistanceTest : public ::testing::Test {};

TEST_F(SquaredDistanceTest, Test) {
  const Vector3 v1(1.0, 2.0, 3.0);
  const Vector3 v2(5.0, 5.0, 3.0);
  const double squared_distance = 16 + 9 + 0;
  EXPECT_DOUBLE_EQ(squared_distance, (details::SquaredDistance<Vector3, 3>()(v1, v2)));
}

// Tests maliput::math::details::NodeCmp functor.
class NodeCmpTest : public ::testing::Test {
 public:
  // A Node implementation to be used by NodeCmp functor and MakeKDTree method.
  template <typename Coordinate>
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
    // @returns The left sub-node.
    const Node* get_left() const { return left_; }
    // @returns The right sub-node.
    const Node* get_right() const { return right_; }

   private:
    Coordinate point_;
    Node* left_{nullptr};
    Node* right_{nullptr};
  };
};

// Tests NodeCmp functor.
TEST_F(NodeCmpTest, Test) {
  const Node lhs{Vector3(1.0, 2.0, 3.0)};
  const Node rhs{Vector3(0.0, 3.0, 3.0)};
  {  // lhs[index] > rhs[index] --> false
    const int index = 0;
    const details::NodeCmp<3> dut{index};
    EXPECT_FALSE(dut(lhs, rhs));
  }
  {  // lhs[index] < rhs[index] --> true
    const int index = 1;
    const details::NodeCmp<3> dut{index};
    EXPECT_TRUE(dut(lhs, rhs));
  }
  {  // lhs[index] == rhs[index] --> true
    const int index = 2;
    const details::NodeCmp<3> dut{index};
    EXPECT_FALSE(dut(lhs, rhs));
  }
}

// Verifies that maliput::math::details::MakeKDTree() works as expected.
// The test case is a 3-dimensional tree with a data structure size of 15.
// The values for each level of the tree(deep) are verified by hand.
class MakeKDTreeTest : public NodeCmpTest {
 public:
  std::deque<Node<Vector3>> nodes{
      Vector3{6, 5, 2}, Vector3{1, 8, 9}, Vector3{7, 1, 1}, Vector3{7, 8, 7}, Vector3{7, 3, 6},
      Vector3{2, 1, 4}, Vector3{1, 4, 1}, Vector3{8, 7, 3}, Vector3{6, 2, 8}, Vector3{2, 8, 6},
      Vector3{9, 1, 4}, Vector3{2, 8, 3}, Vector3{8, 1, 4}, Vector3{9, 4, 1}, Vector3{7, 2, 9},
  };

  const std::vector<Vector3> first_level{{7, 2, 9}};
  const std::vector<Vector3> second_level{{6, 5, 2}, {7, 3, 6}};
  const std::vector<Vector3> third_level{{2, 1, 4}, {2, 8, 6}, {9, 1, 4}, {8, 7, 3}};
  const std::vector<Vector3> fourth_level{{1, 4, 1}, {6, 2, 8}, {2, 8, 3}, {1, 8, 9},
                                          {7, 1, 1}, {8, 1, 4}, {9, 4, 1}, {7, 8, 7}};

  // Expects that @p point is located in @p points.
  void ExpectToFind(const Vector3& point, const std::vector<Vector3>& points) {
    auto it = std::find_if(points.begin(), points.end(), [&point](const Vector3& p) { return p == point; });
    EXPECT_NE(it, points.end());
  }
};

// Verifies that the KD tree is constructed correctly.
TEST_F(MakeKDTreeTest, Test) {
  const auto dut = details::MakeKdTree<3, Node<Vector3>, details::NodeCmp<3>>(0, nodes.size(), 0, nodes);
  // First level.
  ExpectToFind(dut->get_coordinate(), first_level);
  // Second level.
  const auto second_level_1 = dut->get_left();
  const auto second_level_2 = dut->get_right();
  ExpectToFind(second_level_1->get_coordinate(), second_level);
  ExpectToFind(second_level_2->get_coordinate(), second_level);
  // Third level.
  const auto third_level_1 = second_level_1->get_left();
  const auto third_level_2 = second_level_1->get_right();
  const auto third_level_3 = second_level_2->get_left();
  const auto third_level_4 = second_level_2->get_right();
  ExpectToFind(third_level_1->get_coordinate(), third_level);
  ExpectToFind(third_level_2->get_coordinate(), third_level);
  ExpectToFind(third_level_3->get_coordinate(), third_level);
  ExpectToFind(third_level_4->get_coordinate(), third_level);
  // Fourth level.
  const auto fourth_level_1 = third_level_1->get_left();
  const auto fourth_level_2 = third_level_1->get_right();
  const auto fourth_level_3 = third_level_2->get_left();
  const auto fourth_level_4 = third_level_2->get_right();
  const auto fourth_level_5 = third_level_3->get_left();
  const auto fourth_level_6 = third_level_3->get_right();
  const auto fourth_level_7 = third_level_4->get_left();
  const auto fourth_level_8 = third_level_4->get_right();
  ExpectToFind(fourth_level_1->get_coordinate(), fourth_level);
  ExpectToFind(fourth_level_2->get_coordinate(), fourth_level);
  ExpectToFind(fourth_level_3->get_coordinate(), fourth_level);
  ExpectToFind(fourth_level_4->get_coordinate(), fourth_level);
  ExpectToFind(fourth_level_5->get_coordinate(), fourth_level);
  ExpectToFind(fourth_level_6->get_coordinate(), fourth_level);
  ExpectToFind(fourth_level_7->get_coordinate(), fourth_level);
  ExpectToFind(fourth_level_8->get_coordinate(), fourth_level);
}

// Tests KDTree class.
class KDTreeTest : public ::testing::Test {};

// Evaluates constructors.
TEST_F(KDTreeTest, Constructor) {
  const std::vector<Vector3> points{};
  ASSERT_THROW((KDTree<Vector3, 3>(points.begin(), points.end())), maliput::common::assertion_error);
  ASSERT_NO_THROW((KDTree<Vector3, 3>((std::deque<Vector3>{{3, 6, 2}}))));
}

// Evaluates KDTree::Nearest method.
TEST_F(KDTreeTest, NNSearch) {
  const double kTolerance{1e-12};
  const std::vector<Vector3> points{{6, 5, 2}, {1, 8, 9}, {7, 1, 1}, {7, 8, 7}, {7, 3, 6},
                                    {2, 1, 4}, {1, 4, 1}, {8, 7, 3}, {6, 2, 8}, {2, 8, 6},
                                    {9, 1, 4}, {2, 8, 3}, {8, 1, 4}, {9, 4, 1}, {7, 2, 9}};
  const KDTree<Vector3, 3> dut{points.begin(), points.end()};
  const Vector3 point{3., 3., 3.};
  const Vector3 expected_point{2., 1., 4.};

  // Use Nearest() without tolerance selection.
  const auto nearest_point = dut.Nearest(point);
  // Tests point being returned.
  EXPECT_DOUBLE_EQ(expected_point.x(), nearest_point.x());
  EXPECT_DOUBLE_EQ(expected_point.y(), nearest_point.y());
  EXPECT_DOUBLE_EQ(expected_point.z(), nearest_point.z());

  // Use Nearest() without tolerance selection.
  const auto nearest_point_with_tol = dut.Nearest(point, kTolerance);
  // Tests point being returned.
  EXPECT_DOUBLE_EQ(expected_point.x(), nearest_point_with_tol.x());
  EXPECT_DOUBLE_EQ(expected_point.y(), nearest_point_with_tol.y());
  EXPECT_DOUBLE_EQ(expected_point.z(), nearest_point_with_tol.z());
}

// Custom Coordinate class for testing the KDTree class.
// Inherits from Vector3 and adds a id field for uniquely identifying each point.
class UniquePoint : public Vector3 {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniquePoint)
  UniquePoint() = default;
  UniquePoint(double x, double y, double z, unsigned int id) : Vector3(x, y, z), id_(id) {}
  unsigned int id() const { return id_; }

 private:
  unsigned int id_{};
};

// Tests KDTree with a custom Coordinate class and for a large number of points.
class KDTreeExtendedTest : public ::testing::Test {
 public:
  // Convenient method for getting unique ids.
  static unsigned int GetUniqueId() {
    static unsigned int id = 0;
    return ++id;
  }
  // Returns a vector of @p number_of_points UniquePoints obtained randomly, with a uniform distribution, within a range
  // delimited by @p range_min and @p range_max
  static std::vector<UniquePoint> GetRandomPoints(int number_of_points, double range_min, double range_max) {
    std::random_device rd;                                               // obtain a random number from hardware
    std::mt19937 gen(rd());                                              // seed the generator
    std::uniform_real_distribution<double> distr(range_min, range_max);  // define the range

    std::vector<UniquePoint> points;
    for (int j = 0; j < number_of_points; ++j) {
      points.push_back({distr(gen), distr(gen), distr(gen), GetUniqueId()});
    }
    return points;
  }

  // Obtains the nearest point to @p point in @p points using brute-force algorithm.
  template <typename T>
  static T BruteForceNNSearch(const T& point, const std::vector<T>& points) {
    double min_dist = std::numeric_limits<double>::infinity();
    T min_point{};
    for (auto& p : points) {
      double dist = (point - p).norm();
      if (dist < min_dist) {
        min_dist = dist;
        min_point = p;
      }
    }
    return min_point;
  }
};

// Tests KDTree with a custom Coordinate class and for a large number of points.
TEST_F(KDTreeExtendedTest, RandomData) {
  auto points = GetRandomPoints(1000000, -1000, 1000);
  KDTree<UniquePoint, 3> tree{points.begin(), points.end()};

  const UniquePoint evaluation_point{50., 50., 50., 0 /* id is unused here */};
  const UniquePoint expected_point = BruteForceNNSearch<>(evaluation_point, points);

  auto dut = tree.Nearest(evaluation_point);

  EXPECT_EQ(expected_point.id(), dut.id());
  EXPECT_DOUBLE_EQ(expected_point.x(), dut.x());
  EXPECT_DOUBLE_EQ(expected_point.y(), dut.y());
  EXPECT_DOUBLE_EQ(expected_point.z(), dut.z());
}

}  // namespace
}  // namespace math
}  // namespace maliput
