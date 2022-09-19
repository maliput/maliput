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

// Tests maliput::math::details::Node class.
class NodeTest : public ::testing::Test {};

TEST_F(NodeTest, Test) {
  const Vector3 left_coord{1., 2., 3.};
  const Vector3 right_coord{4., 5., 6.};
  const Vector3 parent_coord{7., 8., 9.};
  const AxisAlignedBox expected_region{Vector3{1., 2., 3.}, Vector3{10., 11., 11.}};
  details::Node<Vector3, AxisAlignedBox> left{left_coord};
  details::Node<Vector3, AxisAlignedBox> right{right_coord};
  details::Node<Vector3, AxisAlignedBox> parent{parent_coord};
  parent.set_index(14.);
  parent.set_left(&left);
  parent.set_right(&right);
  parent.set_region(std::make_unique<AxisAlignedBox>(expected_region));
  left.set_parent(&parent);
  EXPECT_EQ(left.get_coordinate(), left_coord);
  EXPECT_EQ(left.get_parent(), &parent);
  EXPECT_EQ(parent.get_index(), 14.);
  EXPECT_EQ(parent.get_parent(), nullptr);
  EXPECT_EQ(parent.get_left(), &left);
  EXPECT_EQ(parent.get_right(), &right);
  EXPECT_EQ(parent.get_region().max_corner(), expected_region.max_corner());
  EXPECT_EQ(parent.get_region().min_corner(), expected_region.min_corner());
}

// Tests maliput::math::details::NodeCmp functor.
class NodeCmpTest : public ::testing::Test {};

// Tests NodeCmp functor.
TEST_F(NodeCmpTest, Test) {
  const details::Node<Vector3, BoundingRegion<Vector3>> lhs{Vector3(1.0, 2.0, 3.0)};
  const details::Node<Vector3, BoundingRegion<Vector3>> rhs{Vector3(0.0, 3.0, 3.0)};
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

// Verifies that maliput::math::details::Initialize3dRegions() works as expected.
// The test case is a 3-dimensional tree with a data structure size of 15.
// The values for each level of the tree(deep) are verified by hand.
class InitializeRegionsTest : public ::testing::Test {
 public:
  static constexpr double kInfinite = std::numeric_limits<double>::infinity();

  void SetUp() override {
    nodes_.emplace_back(Vector3{6, 5, 2});
    nodes_.emplace_back(Vector3{1, 8, 9});
    nodes_.emplace_back(Vector3{7, 1, 1});
    nodes_.emplace_back(Vector3{7, 8, 7});
    nodes_.emplace_back(Vector3{7, 3, 6});
    nodes_.emplace_back(Vector3{2, 1, 4});
    nodes_.emplace_back(Vector3{1, 4, 1});
    nodes_.emplace_back(Vector3{8, 7, 3});
    nodes_.emplace_back(Vector3{6, 2, 8});
    nodes_.emplace_back(Vector3{2, 8, 6});
    nodes_.emplace_back(Vector3{9, 1, 4});
    nodes_.emplace_back(Vector3{2, 8, 3});
    nodes_.emplace_back(Vector3{8, 1, 4});
    nodes_.emplace_back(Vector3{9, 4, 1});
    nodes_.emplace_back(Vector3{7, 2, 9});

    root_ = details::MakeKdTree<3, details::Node<Vector3, AxisAlignedBox>, details::NodeCmp<3>>(0, nodes_.size(), 0,
                                                                                                nodes_);
  }
  // Coordinate nodes for each depth level in the kdtree.
  const std::vector<Vector3> first_level{{7, 2, 9}};
  const std::vector<Vector3> second_level{{6, 5, 2}, {7, 3, 6}};
  const std::vector<Vector3> third_level{{2, 1, 4}, {2, 8, 6}, {9, 1, 4}, {8, 7, 3}};
  const std::vector<Vector3> fourth_level{{1, 4, 1}, {6, 2, 8}, {2, 8, 3}, {1, 8, 9},
                                          {7, 1, 1}, {8, 1, 4}, {9, 4, 1}, {7, 8, 7}};

  // Regions for each level of the kdtree.
  const AxisAlignedBox first_level_region{{-kInfinite, -kInfinite, -kInfinite}, {kInfinite, kInfinite, kInfinite}};
  const AxisAlignedBox second_level_region_1{{-kInfinite, -kInfinite, -kInfinite}, {7, kInfinite, kInfinite}};
  const AxisAlignedBox second_level_region_2{{7, -kInfinite, -kInfinite}, {kInfinite, kInfinite, kInfinite}};
  const AxisAlignedBox third_level_region_1{{-kInfinite, -kInfinite, -kInfinite}, {7, 5, kInfinite}};
  const AxisAlignedBox third_level_region_2{{-kInfinite, 5, -kInfinite}, {7, kInfinite, kInfinite}};
  const AxisAlignedBox third_level_region_3{{7, -kInfinite, -kInfinite}, {kInfinite, 3, kInfinite}};
  const AxisAlignedBox third_level_region_4{{7, 3, -kInfinite}, {kInfinite, kInfinite, kInfinite}};
  const AxisAlignedBox fourth_level_region_1{{-kInfinite, -kInfinite, -kInfinite}, {7, 5, 4}};
  const AxisAlignedBox fourth_level_region_2{{-kInfinite, -kInfinite, 4}, {7, 5, kInfinite}};
  const AxisAlignedBox fourth_level_region_3{{-kInfinite, 5, -kInfinite}, {7, kInfinite, 6}};
  const AxisAlignedBox fourth_level_region_4{{-kInfinite, 5, 6}, {7, kInfinite, kInfinite}};
  const AxisAlignedBox fourth_level_region_5{{7, -kInfinite, -kInfinite}, {kInfinite, 3, 4}};
  const AxisAlignedBox fourth_level_region_6{{7, -kInfinite, 4}, {kInfinite, 3, kInfinite}};
  const AxisAlignedBox fourth_level_region_7{{7, 3, -kInfinite}, {kInfinite, kInfinite, 3}};
  const AxisAlignedBox fourth_level_region_8{{7, 3, 3}, {kInfinite, kInfinite, kInfinite}};

  std::deque<details::Node<Vector3, AxisAlignedBox>> nodes_;
  details::Node<Vector3, AxisAlignedBox>* root_{nullptr};
};

TEST_F(InitializeRegionsTest, Test) {
  root_->set_region(std::make_unique<AxisAlignedBox>(Vector3{{-kInfinite, -kInfinite, -kInfinite}},
                                                     Vector3{{kInfinite, kInfinite, kInfinite}}));
  if (root_->get_left() != nullptr) details::Initialize3dRegions(true, root_->get_left());
  if (root_->get_right() != nullptr) details::Initialize3dRegions(false, root_->get_right());

  // Obtain the nodes.
  // Note that the nodes_ variable got reordered after calling MakeKdTree method.
  const details::Node<Vector3, AxisAlignedBox>* second_level_node_1 = &nodes_[3];   // root_->get_left();
  const details::Node<Vector3, AxisAlignedBox>* second_level_node_2 = &nodes_[11];  // root_->get_right();
  const details::Node<Vector3, AxisAlignedBox>* third_level_node_1 = &nodes_[1];    // second_level_node_1->get_left();
  const details::Node<Vector3, AxisAlignedBox>* third_level_node_2 = &nodes_[5];    // second_level_node_1->get_right();
  const details::Node<Vector3, AxisAlignedBox>* third_level_node_3 = &nodes_[9];    // second_level_node_2->get_left();
  const details::Node<Vector3, AxisAlignedBox>* third_level_node_4 = &nodes_[13];   // second_level_node_2->get_right();
  const details::Node<Vector3, AxisAlignedBox>* fourth_level_node_1 = &nodes_[0];   // third_level_node_1->get_left();
  const details::Node<Vector3, AxisAlignedBox>* fourth_level_node_2 = &nodes_[2];   // third_level_node_1->get_right();
  const details::Node<Vector3, AxisAlignedBox>* fourth_level_node_3 = &nodes_[4];   // third_level_node_2->get_left();
  const details::Node<Vector3, AxisAlignedBox>* fourth_level_node_4 = &nodes_[6];   // third_level_node_2->get_right();
  const details::Node<Vector3, AxisAlignedBox>* fourth_level_node_5 = &nodes_[8];   // third_level_node_3->get_left();
  const details::Node<Vector3, AxisAlignedBox>* fourth_level_node_6 = &nodes_[10];  // third_level_node_3->get_right();
  const details::Node<Vector3, AxisAlignedBox>* fourth_level_node_7 = &nodes_[12];  // third_level_node_4->get_left();
  const details::Node<Vector3, AxisAlignedBox>* fourth_level_node_8 = &nodes_[14];  // third_level_node_4->get_right();

  // Verify the regions.
  // second level
  EXPECT_EQ(second_level_node_1->get_region().min_corner(), second_level_region_1.min_corner());
  EXPECT_EQ(second_level_node_1->get_region().max_corner(), second_level_region_1.max_corner());
  EXPECT_EQ(second_level_node_2->get_region().min_corner(), second_level_region_2.min_corner());
  EXPECT_EQ(second_level_node_2->get_region().max_corner(), second_level_region_2.max_corner());
  // third level
  EXPECT_EQ(third_level_node_1->get_region().min_corner(), third_level_region_1.min_corner());
  EXPECT_EQ(third_level_node_1->get_region().max_corner(), third_level_region_1.max_corner());
  EXPECT_EQ(third_level_node_2->get_region().min_corner(), third_level_region_2.min_corner());
  EXPECT_EQ(third_level_node_2->get_region().max_corner(), third_level_region_2.max_corner());
  EXPECT_EQ(third_level_node_3->get_region().min_corner(), third_level_region_3.min_corner());
  EXPECT_EQ(third_level_node_3->get_region().max_corner(), third_level_region_3.max_corner());
  EXPECT_EQ(third_level_node_4->get_region().min_corner(), third_level_region_4.min_corner());
  EXPECT_EQ(third_level_node_4->get_region().max_corner(), third_level_region_4.max_corner());
  // fourth level
  EXPECT_EQ(fourth_level_node_1->get_region().min_corner(), fourth_level_region_1.min_corner());
  EXPECT_EQ(fourth_level_node_1->get_region().max_corner(), fourth_level_region_1.max_corner());
  EXPECT_EQ(fourth_level_node_2->get_region().min_corner(), fourth_level_region_2.min_corner());
  EXPECT_EQ(fourth_level_node_2->get_region().max_corner(), fourth_level_region_2.max_corner());
  EXPECT_EQ(fourth_level_node_3->get_region().min_corner(), fourth_level_region_3.min_corner());
  EXPECT_EQ(fourth_level_node_3->get_region().max_corner(), fourth_level_region_3.max_corner());
  EXPECT_EQ(fourth_level_node_4->get_region().min_corner(), fourth_level_region_4.min_corner());
  EXPECT_EQ(fourth_level_node_4->get_region().max_corner(), fourth_level_region_4.max_corner());
  EXPECT_EQ(fourth_level_node_5->get_region().min_corner(), fourth_level_region_5.min_corner());
  EXPECT_EQ(fourth_level_node_5->get_region().max_corner(), fourth_level_region_5.max_corner());
  EXPECT_EQ(fourth_level_node_6->get_region().min_corner(), fourth_level_region_6.min_corner());
  EXPECT_EQ(fourth_level_node_6->get_region().max_corner(), fourth_level_region_6.max_corner());
  EXPECT_EQ(fourth_level_node_7->get_region().min_corner(), fourth_level_region_7.min_corner());
  EXPECT_EQ(fourth_level_node_7->get_region().max_corner(), fourth_level_region_7.max_corner());
  EXPECT_EQ(fourth_level_node_8->get_region().min_corner(), fourth_level_region_8.min_corner());
  EXPECT_EQ(fourth_level_node_8->get_region().max_corner(), fourth_level_region_8.max_corner());
}

// Verifies that maliput::math::details::MakeKDTree() works as expected.
// The test case is a 3-dimensional tree with a data structure size of 15.
// The values for each level of the tree(deep) are verified by hand.
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
  // First level.
  ExpectToFind(root_->get_coordinate(), first_level);
  // Second level.
  const auto second_level_1 = root_->get_left();
  const auto second_level_2 = root_->get_right();
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
class KDTreeTest : public ::testing::Test {
 public:
  const std::vector<Vector3> points{{6, 5, 2}, {1, 8, 9}, {7, 1, 1}, {7, 8, 7}, {7, 3, 6},
                                    {2, 1, 4}, {1, 4, 1}, {8, 7, 3}, {6, 2, 8}, {2, 8, 6},
                                    {9, 1, 4}, {2, 8, 3}, {8, 1, 4}, {9, 4, 1}, {7, 2, 9}};
  KDTree3D<Vector3> dut{points.begin(), points.end()};
};

// Evaluates constructors.
TEST_F(KDTreeTest, Constructor) {
  const std::vector<Vector3> empty_points{};
  ASSERT_THROW((KDTree<Vector3, 3>(empty_points.begin(), empty_points.end())), maliput::common::assertion_error);
  ASSERT_NO_THROW((KDTree<Vector3, 3>((std::deque<Vector3>{{3, 6, 2}}))));
}

// Evaluates KDTree::Nearest method.
TEST_F(KDTreeTest, NNSearch) {
  const double kTolerance{1e-12};
  const Vector3 point{3., 3., 3.};
  const Vector3 expected_point{2., 1., 4.};

  // Use nearest_point() without tolerance selection.
  const auto nearest_point = dut.nearest_point(point);
  // Tests point being returned.
  EXPECT_DOUBLE_EQ(expected_point.x(), nearest_point.x());
  EXPECT_DOUBLE_EQ(expected_point.y(), nearest_point.y());
  EXPECT_DOUBLE_EQ(expected_point.z(), nearest_point.z());

  // Use nearest_point() without tolerance selection.
  const auto nearest_point_with_tol = dut.nearest_point(point, kTolerance);
  // Tests point being returned.
  EXPECT_DOUBLE_EQ(expected_point.x(), nearest_point_with_tol.x());
  EXPECT_DOUBLE_EQ(expected_point.y(), nearest_point_with_tol.y());
  EXPECT_DOUBLE_EQ(expected_point.z(), nearest_point_with_tol.z());
}

// Evaluates KDTree3D::RangeSearch method.
TEST_F(KDTreeTest, RangeSearchReturnsAllPointsWithEnclosingRegion) {
  AxisAlignedBox search_region{{1., 1., 1.}, {9., 9., 9.}};
  const auto contained_points = dut.RangeSearch(search_region);
  EXPECT_EQ(contained_points.size(), points.size());
  for (const auto& expected_point : points) {
    std::find_if(contained_points.begin(), contained_points.end(),
                 [&expected_point](const Vector3* p) { return *p == expected_point; });
  }
}

// Custom Coordinate class for testing the KDTree class.
// Inherits from Vector3 and adds a id field for uniquely identifying each point.
class UniquePoint : public Vector3 {
 public:
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(UniquePoint)
  UniquePoint() = default;
  UniquePoint(double x, double y, double z, unsigned int id = 0) : Vector3(x, y, z), id_(id) {}
  unsigned int id() const { return id_; }

  bool operator==(const UniquePoint& other) const { return id_ == other.id_ && Vector3::operator==(other); }

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
  static std::deque<UniquePoint> GetRandomPoints(int number_of_points, double range_min, double range_max) {
    std::mt19937 gen(127);                                               // seed the generator
    std::uniform_real_distribution<double> distr(range_min, range_max);  // define the range

    std::deque<UniquePoint> points;
    for (int j = 0; j < number_of_points; ++j) {
      points.push_back({distr(gen), distr(gen), distr(gen), GetUniqueId()});
    }
    return points;
  }

  // Obtains the nearest point to @p point in @p points using brute-force algorithm.
  template <typename T>
  static T BruteForceNNSearch(const T& point, const std::deque<T>& points) {
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

  // Performs a brute-force-based range query on @p search_region out of the @p points.
  template <typename T>
  static std::deque<T> BruteForceRangeSearch(const AxisAlignedBox& search_region, const std::deque<T>& points) {
    std::deque<T> res;
    for (auto& p : points) {
      Vector3 p_in_vector_3{p.x(), p.y(), p.z()};
      if (search_region.Contains(p_in_vector_3)) {
        res.push_back(p);
      }
    }
    return res;
  }
};

// Tests KDTree with a custom Coordinate class and for a large number of points.
TEST_F(KDTreeExtendedTest, RandomData) {
  const auto points = GetRandomPoints(1000000, -1000, 1000);
  KDTree3D<UniquePoint> dut{points.begin(), points.end()};

  const UniquePoint evaluation_point{50., 50., 50., 0 /* id is unused here */};
  const UniquePoint expected_point = BruteForceNNSearch<>(evaluation_point, points);

  auto nearest = dut.nearest_point(evaluation_point);
  EXPECT_EQ(expected_point.id(), nearest.id());
  EXPECT_DOUBLE_EQ(expected_point.x(), nearest.x());
  EXPECT_DOUBLE_EQ(expected_point.y(), nearest.y());
  EXPECT_DOUBLE_EQ(expected_point.z(), nearest.z());

  const AxisAlignedBox evaluation_range{{-250., -250., -250.}, {250., 250., 250.}};
  const auto expected_range_search = BruteForceRangeSearch(evaluation_range, points);

  auto range_search = dut.RangeSearch(evaluation_range);
  ASSERT_EQ(expected_range_search.size(), range_search.size());
  for (const auto& expected_value : expected_range_search) {
    const auto it = std::find_if(range_search.begin(), range_search.end(),
                                 [&expected_value](const auto& value) { return value->id() == expected_value.id(); });
    EXPECT_TRUE(it != range_search.end());
  }
  for (const auto& value : range_search) {
    const Vector3 coord{(*value)[0], (*value)[1], (*value)[2]};
    EXPECT_TRUE(evaluation_range.Contains(coord)) << "Coordinate isn't within the search region: " << coord;
  }
}

}  // namespace
}  // namespace math
}  // namespace maliput
