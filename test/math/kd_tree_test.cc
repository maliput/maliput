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
#include "maliput/math/axis_aligned_box.h"
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
    Node(const Node& other) : point_(other.point_), left_(other.left_), right_(other.right_), parent_(other.parent_)
       {
        auto aa_box = dynamic_cast<AxisAlignedBox*>(other.region_.get());
      if(aa_box != nullptr)
        region_ = (std::make_unique<AxisAlignedBox>(aa_box->min_corner(), aa_box->max_corner()));
      }

    Node(Coordinate&& point) : point_(std::forward<Coordinate>(point)) {}

    Node() = default;
    Node(Node&& other){
      point_ = other.point_;
      index_ = other.index_;
      left_ = other.left_;
      right_ = other.right_;
      auto aa_box = dynamic_cast<AxisAlignedBox*>(other.region_.get());
      if(aa_box != nullptr)
      region_ = (std::make_unique<AxisAlignedBox>(aa_box->min_corner(), aa_box->max_corner()));
    }

    Node& operator=(Node&& other){
      point_ = other.point_;
      index_ = other.index_;
      left_ = other.left_;
      right_ = other.right_;
        auto aa_box = dynamic_cast<AxisAlignedBox*>(other.region_.get());
      if(aa_box != nullptr)
      region_ = (std::make_unique<AxisAlignedBox>(aa_box->min_corner(), aa_box->max_corner()));
      return *this;
    }

    // Returns the point that the node represents.
    const Coordinate& get_coordinate() const { return point_; }

    void set_left(Node* left) { left_ = left; }
    void set_right(Node* right) { right_ = right; }
    void set_region(std::unique_ptr<BoundingRegion<Coordinate>> region) { region_ = std::move(region); }
    void set_parent(Node* parent) { parent_ = parent; }
    void set_index(std::size_t index) { index_ = index; }
    Node* get_left() { return left_; }
    Node* get_right() { return right_; }
    Node const* get_left() const { return left_; }
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
    std::unique_ptr<BoundingRegion<Coordinate>> region_{};
  };
};

// Tests NodeCmp functor.
TEST_F(NodeCmpTest, Test) {
  const Node<Vector3> lhs{Vector3(1.0, 2.0, 3.0)};
  const Node<Vector3> rhs{Vector3(0.0, 3.0, 3.0)};
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
class InitializeRegionsTest : public NodeCmpTest {
 public:

  std::deque<Node<Vector3>> nodes{
      Node<Vector3>{Vector3{6, 5, 2}}, Node<Vector3>{Vector3{1, 8, 9}}, Node<Vector3>{Vector3{7, 1, 1}}, Node<Vector3>{Vector3{7, 8, 7}}, Node<Vector3>{Vector3{7, 3, 6}},
      Node<Vector3>{Vector3{2, 1, 4}}, Node<Vector3>{Vector3{1, 4, 1}}, Node<Vector3>{Vector3{8, 7, 3}}, Node<Vector3>{Vector3{6, 2, 8}}, Node<Vector3>{Vector3{2, 8, 6}},
      Node<Vector3>{Vector3{9, 1, 4}}, Node<Vector3>{Vector3{2, 8, 3}}, Node<Vector3>{Vector3{8, 1, 4}}, Node<Vector3>{Vector3{9, 4, 1}}, Node<Vector3>{Vector3{7, 2, 9}},
  };

  const std::vector<Vector3> first_level{{7, 2, 9}};
  const std::vector<Vector3> second_level{{6, 5, 2}, {7, 3, 6}};
  const std::vector<Vector3> third_level{{2, 1, 4}, {2, 8, 6}, {9, 1, 4}, {8, 7, 3}};
  const std::vector<Vector3> fourth_level{{1, 4, 1}, {6, 2, 8}, {2, 8, 3}, {1, 8, 9},
                                          {7, 1, 1}, {8, 1, 4}, {9, 4, 1}, {7, 8, 7}};

};

TEST_F(InitializeRegionsTest, Test) {
  const auto root = details::MakeKdTree<3, Node<Vector3>, details::NodeCmp<3>>(0, nodes.size(), 0, nodes);
  root->set_region(std::make_unique<AxisAlignedBox>(
    Vector3{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()},
    Vector3{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}));
  if(root->get_left() != nullptr)details::Initialize3dRegions(true, root->get_left());
  if(root->get_right() != nullptr)details::Initialize3dRegions(false, root->get_right());

  // First level region
  auto first_level_region = dynamic_cast<const AxisAlignedBox&>(root->get_region());
  // Second level region
  auto second_level_node_1 = root->get_left();
  auto second_level_node_2 = root->get_right();
  auto second_level_region_1 = dynamic_cast<const AxisAlignedBox&>(second_level_node_1->get_region());
  auto second_level_region_2 = dynamic_cast<const AxisAlignedBox&>(second_level_node_2->get_region());
  std::cout << "first level region: Min corner: " << first_level_region.min_corner() << std::endl;
  std::cout << "first level region: max corner: " << first_level_region.max_corner() << std::endl;
  EXPECT_EQ(first_level_region.min_corner(), Vector3(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()));
  EXPECT_EQ(first_level_region.max_corner(), Vector3(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
  std::cout << "second level region 1: Min corner: " << second_level_region_1.min_corner() << std::endl;
  std::cout << "second level region 1: max corner: " << second_level_region_1.max_corner() << std::endl;
  EXPECT_EQ(second_level_region_1.min_corner(), Vector3(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()));
  EXPECT_EQ(second_level_region_1.max_corner(), Vector3(7, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
  std::cout << "second level region 2: Min corner: " << second_level_region_2.min_corner() << std::endl;
  std::cout << "second level region 2: max corner: " << second_level_region_2.max_corner() << std::endl;
  EXPECT_EQ(second_level_region_2.min_corner(), Vector3(7, -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()));
  EXPECT_EQ(second_level_region_2.max_corner(), Vector3(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()));
  // Third level region
  auto third_level_node_1 = second_level_node_1->get_left();
  auto third_level_node_2 = second_level_node_1->get_right();
  auto third_level_node_3 = second_level_node_2->get_left();
  auto third_level_node_4 = second_level_node_2->get_right();
  auto third_level_region_1 = dynamic_cast<const AxisAlignedBox&>(third_level_node_1->get_region());
  auto third_level_region_2 = dynamic_cast<const AxisAlignedBox&>(third_level_node_2->get_region());
  auto third_level_region_3 = dynamic_cast<const AxisAlignedBox&>(third_level_node_3->get_region());
  auto third_level_region_4 = dynamic_cast<const AxisAlignedBox&>(third_level_node_4->get_region());
  std::cout << "third level region 1: Min corner: " << third_level_region_1.min_corner() << std::endl;
  std::cout << "third level region 1: max corner: " << third_level_region_1.max_corner() << std::endl;
  std::cout << "third level region 2: Min corner: " << third_level_region_2.min_corner() << std::endl;
  std::cout << "third level region 2: max corner: " << third_level_region_2.max_corner() << std::endl;
  std::cout << "third level region 3: Min corner: " << third_level_region_3.min_corner() << std::endl;
  std::cout << "third level region 3: max corner: " << third_level_region_3.max_corner() << std::endl;
  std::cout << "third level region 4: Min corner: " << third_level_region_4.min_corner() << std::endl;
  std::cout << "third level region 4: max corner: " << third_level_region_4.max_corner() << std::endl;
  // Fourth level region
  auto fourth_level_node_1 = third_level_node_1->get_left();
  auto fourth_level_node_2 = third_level_node_1->get_right();
  auto fourth_level_node_3 = third_level_node_2->get_left();
  auto fourth_level_node_4 = third_level_node_2->get_right();
  auto fourth_level_node_5 = third_level_node_3->get_left();
  auto fourth_level_node_6 = third_level_node_3->get_right();
  auto fourth_level_node_7 = third_level_node_4->get_left();
  auto fourth_level_node_8 = third_level_node_4->get_right();
  auto fourth_level_region_1 = dynamic_cast<const AxisAlignedBox&>(fourth_level_node_1->get_region());
  auto fourth_level_region_2 = dynamic_cast<const AxisAlignedBox&>(fourth_level_node_2->get_region());
  auto fourth_level_region_3 = dynamic_cast<const AxisAlignedBox&>(fourth_level_node_3->get_region());
  auto fourth_level_region_4 = dynamic_cast<const AxisAlignedBox&>(fourth_level_node_4->get_region());
  auto fourth_level_region_5 = dynamic_cast<const AxisAlignedBox&>(fourth_level_node_5->get_region());
  auto fourth_level_region_6 = dynamic_cast<const AxisAlignedBox&>(fourth_level_node_6->get_region());
  auto fourth_level_region_7 = dynamic_cast<const AxisAlignedBox&>(fourth_level_node_7->get_region());
  auto fourth_level_region_8 = dynamic_cast<const AxisAlignedBox&>(fourth_level_node_8->get_region());
  
  std::cout << "fourth level region 1: Min corner: " << fourth_level_region_1.min_corner() << std::endl;
  std::cout << "fourth level region 1: max corner: " << fourth_level_region_1.max_corner() << std::endl;
  std::cout << "fourth level region 2: Min corner: " << fourth_level_region_2.min_corner() << std::endl;
  std::cout << "fourth level region 2: max corner: " << fourth_level_region_2.max_corner() << std::endl;
  std::cout << "fourth level region 3: Min corner: " << fourth_level_region_3.min_corner() << std::endl;
  std::cout << "fourth level region 3: max corner: " << fourth_level_region_3.max_corner() << std::endl;
  std::cout << "fourth level region 4: Min corner: " << fourth_level_region_4.min_corner() << std::endl;
  std::cout << "fourth level region 4: max corner: " << fourth_level_region_4.max_corner() << std::endl;
  std::cout << "fourth level region 5: Min corner: " << fourth_level_region_5.min_corner() << std::endl;
  std::cout << "fourth level region 5: max corner: " << fourth_level_region_5.max_corner() << std::endl;
  std::cout << "fourth level region 6: Min corner: " << fourth_level_region_6.min_corner() << std::endl;
  std::cout << "fourth level region 6: max corner: " << fourth_level_region_6.max_corner() << std::endl;
  std::cout << "fourth level region 7: Min corner: " << fourth_level_region_7.min_corner() << std::endl;
  std::cout << "fourth level region 7: max corner: " << fourth_level_region_7.max_corner() << std::endl;
  std::cout << "fourth level region 8: Min corner: " << fourth_level_region_8.min_corner() << std::endl;
  std::cout << "fourth level region 8: max corner: " << fourth_level_region_8.max_corner() << std::endl;
}

class MakeKDTreeTest : public InitializeRegionsTest {
 public:
  // Expects that @p point is located in @p points.
  void ExpectToFind(const Vector3& point, const std::vector<Vector3>& points) {
    auto it = std::find_if(points.begin(), points.end(), [&point](const Vector3& p) { return p == point; });
    EXPECT_NE(it, points.end());
  }

};

// Verifies that the KD tree is constructed correctly.
TEST_F(MakeKDTreeTest, MakeKDTree) {
   auto dut = details::MakeKdTree<3, Node<Vector3>, details::NodeCmp<3>>(0, nodes.size(), 0, nodes);
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

// Evaluates KDTree::Nearest method.
TEST_F(KDTreeTest, RangeSearch) {
  // const double kTolerance{1e-12};
  const std::vector<Vector3> points{{6, 5, 2}, {1, 8, 9}, {7, 1, 1}, {7, 8, 7}, {7, 3, 6},
                                    {2, 1, 4}, {1, 4, 1}, {8, 7, 3}, {6, 2, 8}, {2, 8, 6},
                                    {9, 1, 4}, {2, 8, 3}, {8, 1, 4}, {9, 4, 1}, {7, 2, 9}};
  KDTree3D<Vector3> dut{points.begin(), points.end()};
  dut.InitializeRegions();
  AxisAlignedBox search_region{ {1., 1., 1.}, {9., 9., 9.} };
  const auto contained_points = dut.RangeSearch(search_region);
  for (const auto& point : contained_points) {
    // EXPECT_TRUE(search_region.Contains(*point));
    std::cout << *point << std::endl;
  }
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
