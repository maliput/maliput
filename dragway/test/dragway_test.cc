#include <cmath>
#include <limits>
#include <map>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/check_id_indexing.h"
#include "maliput/test_utilities/eigen_matrix_compare.h"
#include "maliput/test_utilities/maliput_types_compare.h"

#include "dragway/branch_point.h"
#include "dragway/junction.h"
#include "dragway/lane.h"
#include "dragway/road_geometry.h"

namespace maliput {
namespace dragway {
namespace {

// To understand the characteristics of the geometry, consult the
// dragway::Segment and dragway::Lane detailed class overview docs.
class MaliputDragwayLaneTest : public ::testing::Test {
 public:
  MaliputDragwayLaneTest()
      : length_(100.0),
        lane_width_(6.0),
        shoulder_width_(0.5),
        maximum_height_(5.0),
        min_x_(0.),
        max_x_(length_),
        min_y_(-lane_width_ / 2 - shoulder_width_),
        max_y_(lane_width_ / 2 + shoulder_width_),
        min_z_(0.),
        max_z_(maximum_height_) {}

  // Contains expected segment r_min, segment r_max, and y_offset parameters
  // of a Dragway Lane.
  struct ExpectedLaneParameters {
    double y_offset{};
    double segment_r_min{};
    double segment_r_max{};
    double elevation_min{};
    double elevation_max{};
  };

  void MakeDragway(int num_lanes) {
    const api::RoadGeometryId road_geometry_id(std::to_string(num_lanes) + "LaneDragwayRoadGeometry");
    road_geometry_.reset(new RoadGeometry(road_geometry_id, num_lanes, length_, lane_width_, shoulder_width_,
                                          maximum_height_, kLinearTolerance, kAngularTolerance));

    const api::Junction* junction = road_geometry_->junction(0);
    ASSERT_NE(junction, nullptr);
    segment_ = junction->segment(0);
    ASSERT_NE(segment_, nullptr);
    lane_ = dynamic_cast<const Lane*>(segment_->lane(0));
    ASSERT_NE(lane_, nullptr);

    EXPECT_EQ(lane_->length(), length_);
    EXPECT_EQ(segment_->num_lanes(), num_lanes);
  }

  ExpectedLaneParameters GetExpectedLaneParameters(int num_lanes, int lane_index) const {
    ExpectedLaneParameters result{};
    if (num_lanes == 1) {
      result.y_offset = 0.0;
      result.segment_r_min = -3.5;
      result.segment_r_max = 3.5;
    } else if ((num_lanes == 2) && (lane_index == 0)) {
      result.y_offset = -3.0;
      result.segment_r_min = -3.5;
      result.segment_r_max = 9.5;
    } else if ((num_lanes == 2) && (lane_index == 1)) {
      result.y_offset = 3.0;
      result.segment_r_min = -9.5;
      result.segment_r_max = 3.5;
    } else {
      throw std::runtime_error("GetExpectedLaneParameters: bad input");
    }
    result.elevation_min = 0.0;
    result.elevation_max = 5.0;
    return result;
  }

  // Verifies the correctness of the provided `lane`.
  void VerifyLaneCorrectness(const api::Lane* lane, int num_lanes) {
    const ExpectedLaneParameters expected = GetExpectedLaneParameters(num_lanes, lane->index());

    // Tests Lane::lane_bounds().
    for (double s = 0; s < length_; s += length_ / 100) {
      const api::RBounds lane_bounds = lane->lane_bounds(s);
      EXPECT_TRUE(
          api::test::IsRBoundsClose(lane_bounds, api::RBounds(-lane_width_ / 2, lane_width_ / 2), kLinearTolerance));
    }

    EXPECT_EQ(lane->length(), length_);

    // Tests Lane::segment_bounds().
    for (double s = 0; s < length_; s += length_ / 100) {
      const api::RBounds segment_bounds = lane->segment_bounds(s);
      EXPECT_TRUE(api::test::IsRBoundsClose(
          segment_bounds, api::RBounds(expected.segment_r_min, expected.segment_r_max), kLinearTolerance));
    }

    // Tests Lane::elevation_bounds().
    for (double s = 0; s < length_; s += length_ / 10) {
      const api::RBounds segment_bounds = lane->segment_bounds(s);
      const api::HBounds elevation_bounds0 = lane->elevation_bounds(s, segment_bounds.min());
      const api::HBounds elevation_bounds1 = lane->elevation_bounds(s, segment_bounds.max());
      EXPECT_TRUE(api::test::IsHBoundsClose(
          elevation_bounds0, api::HBounds(expected.elevation_min, expected.elevation_max), kLinearTolerance));
      EXPECT_TRUE(api::test::IsHBoundsClose(
          elevation_bounds1, api::HBounds(expected.elevation_min, expected.elevation_max), kLinearTolerance));
    }

    // The following block of test code evaluates methods that take as input a
    // (s, r, h) lane position. Ideally, we want to check that the
    // method-under-test is correct for all combinations of (s, r, h). This,
    // unfortunately, it not possible since there are too many combinations of
    // (s, r, h). Instead we pick points in a grid that spans the (s, r, h)
    // state space and only check those points in the hopes that they are
    // representative of the entire state space.
    const double segment_width = expected.segment_r_max - expected.segment_r_min;
    for (double s = 0; s < length_; s += length_ / 20) {
      for (double r = expected.segment_r_min; r <= expected.segment_r_max; r += segment_width / 20) {
        for (double h = expected.elevation_min; h < expected.elevation_max;
             h += (expected.elevation_max - expected.elevation_min) * 0.1) {
          const api::LanePosition lane_position(s, r, h);

          // Tests Lane::ToGeoPosition().
          const api::GeoPosition geo_position = lane->ToGeoPosition(lane_position);
          const double linear_tolerance = lane->segment()->junction()->road_geometry()->linear_tolerance();
          EXPECT_DOUBLE_EQ(geo_position.x(), s);
          EXPECT_NEAR(geo_position.y(), expected.y_offset + r, linear_tolerance);
          EXPECT_DOUBLE_EQ(geo_position.z(), h);

          // Tests Lane::ToGeoPositionAutoDiff().
          // Seed lane_position with the partial derivatives
          // ∂s/∂s = 1, ∂r/∂s = 0., ∂h/∂s = 0, etc.
          drake::AutoDiffXd s_autodiff{s, drake::Vector3<double>(1., 0., 0.)};
          drake::AutoDiffXd r_autodiff{r, drake::Vector3<double>(0., 1., 0.)};
          drake::AutoDiffXd h_autodiff{h, drake::Vector3<double>(0., 0., 1.)};
          api::LanePositionT<drake::AutoDiffXd> lane_position_ad{s_autodiff, r_autodiff, h_autodiff};

          const api::GeoPositionT<drake::AutoDiffXd> geo_position_ad =
              lane->ToGeoPositionT<drake::AutoDiffXd>(lane_position_ad);
          EXPECT_DOUBLE_EQ(geo_position_ad.x().value(), s);
          EXPECT_NEAR(geo_position_ad.y().value(), expected.y_offset + r, linear_tolerance);
          EXPECT_DOUBLE_EQ(geo_position_ad.z().value(), h);

          // For the s-coordinate, expect the following derivatives for
          // geo_position: ∂x/∂s = 1, ∂y/∂s = 0., ∂z/∂s = 0 (the world x-y-z
          // axis has the same orientation as Dragway's s-r-h axis).
          EXPECT_TRUE(CompareMatrices(drake::Vector3<double>(1., 0., 0.), geo_position_ad.x().derivatives()));
          EXPECT_TRUE(CompareMatrices(drake::Vector3<double>(0., 1., 0.), geo_position_ad.y().derivatives()));
          EXPECT_TRUE(CompareMatrices(drake::Vector3<double>(0., 0., 1.), geo_position_ad.z().derivatives()));

          // Tests Lane::GetOrientation().
          const api::Rotation rotation = lane->GetOrientation(lane_position);
          EXPECT_TRUE(api::test::IsRotationClose(rotation, api::Rotation::FromRpy(0.0, 0.0, 0.0), kAngularTolerance));

          // Tests Lane::EvalMotionDerivatives().
          //
          // The following translational velocities can be any value. We just
          // want to verify that the same values are returned from
          // Lane::EvalMotionDerivatives().
          const double kSigma_v = 1.1;
          const double kRho_v = 2.2;
          const double kEta_v = 3.3;

          const api::LanePosition motion_derivatives =
              lane->EvalMotionDerivatives(lane_position, api::IsoLaneVelocity(kSigma_v, kRho_v, kEta_v));
          EXPECT_TRUE(api::test::IsLanePositionClose(motion_derivatives, api::LanePosition(kSigma_v, kRho_v, kEta_v),
                                                     kLinearTolerance));
        }
      }
    }
  }

  // Verifies that the branches within the provided `lane` are correct. The
  // provided `lane` must be in the provided `road_geometry`.
  void VerifyBranches(const api::Lane* lane, const RoadGeometry* road_geometry) const {
    // Verifies that the same BranchPoint covers both ends of the dragway lane.
    EXPECT_EQ(lane->GetBranchPoint(api::LaneEnd::kStart), lane->GetBranchPoint(api::LaneEnd::kFinish));

    const api::BranchPoint* branch_point = lane->GetBranchPoint(api::LaneEnd::kStart);
    EXPECT_NE(branch_point, nullptr);
    EXPECT_EQ(branch_point->id(), api::BranchPointId(lane->id().string() + "_Branch_Point"));
    EXPECT_EQ(branch_point->road_geometry(), road_geometry);

    // Verifies correctness of the confluent branches.
    {
      const api::LaneEndSet* lane_end_set_start = lane->GetConfluentBranches(api::LaneEnd::kStart);
      const api::LaneEndSet* lane_end_set_finish = lane->GetConfluentBranches(api::LaneEnd::kFinish);
      EXPECT_EQ(lane_end_set_start->size(), 1);
      EXPECT_EQ(lane_end_set_finish->size(), 1);

      const api::LaneEnd& lane_end_start = lane->GetConfluentBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_start.lane, lane);
      EXPECT_EQ(lane_end_start.end, api::LaneEnd::kStart);
      const api::LaneEnd& lane_end_finish = lane->GetConfluentBranches(api::LaneEnd::kFinish)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
      EXPECT_EQ(lane_end_finish.end, api::LaneEnd::kFinish);
    }

    // Verifies correctness of the ongoing branches.
    {
      const api::LaneEndSet* lane_end_set_start = lane->GetOngoingBranches(api::LaneEnd::kStart);
      const api::LaneEndSet* lane_end_set_finish = lane->GetOngoingBranches(api::LaneEnd::kFinish);
      EXPECT_EQ(lane_end_set_start->size(), 1);
      EXPECT_EQ(lane_end_set_finish->size(), 1);

      const api::LaneEnd& lane_end_start = lane->GetOngoingBranches(api::LaneEnd::kStart)->get(0);
      EXPECT_EQ(lane_end_start.lane, lane);
      EXPECT_EQ(lane_end_start.end, api::LaneEnd::kFinish);
      const api::LaneEnd& lane_end_finish = lane->GetOngoingBranches(api::LaneEnd::kFinish)->get(0);
      EXPECT_EQ(lane_end_finish.lane, lane);
      EXPECT_EQ(lane_end_finish.end, api::LaneEnd::kStart);
    }

    // Verifies correctness of the default branches.
    {
      drake::optional<api::LaneEnd> default_start_lane_end = lane->GetDefaultBranch(api::LaneEnd::kStart);
      EXPECT_TRUE(default_start_lane_end);
      EXPECT_EQ(default_start_lane_end->end, api::LaneEnd::kFinish);
      EXPECT_EQ(default_start_lane_end->lane, lane);

      drake::optional<api::LaneEnd> default_finish_lane_end = lane->GetDefaultBranch(api::LaneEnd::kFinish);
      EXPECT_TRUE(default_finish_lane_end);
      EXPECT_EQ(default_finish_lane_end->end, api::LaneEnd::kStart);
      EXPECT_EQ(default_finish_lane_end->lane, lane);
    }
  }

  void PopulateGeoPositionTestCases() {
    /*
      A figure of the one-lane dragway is shown below. The minimum and maximum
      values of the dragway' segment surface are demarcated.

      X
      Y = max_y ^  Y = min_y
      :
      |     :     |
      |     :     |
      --------+-----+-----+---------  X = max_x
      | .   :   . |
      | .   :   . |
      | .   :   . |
      | .  The  . |
      | .Dragway. |
      | .   :   . |
      | .   :   . |
      | .   :   . |
      Y <----------+-----o-----+---------  X = min_x
      |     :     |
      :
      ->|-|<- : ->|-|<-  shoulder_width
      :
      |<--:-->|      lane_width
      V
    */

    // Defines the test case values for x and y. Points both far away and close
    // to the segment area are evaluated, with a focus on edge cases.
    std::vector<double> x_values{min_x_ - 10.,   min_x_ - 1e-10, min_x_,         min_x_ + 1e-10, (max_x_ + min_x_) / 2.,
                                 max_x_ - 1e-10, max_x_,         max_x_ + 1e-10, max_x_ + 10.};
    std::vector<double> y_values{min_y_ - 10.,   min_y_ - 1e-10, min_y_,         min_y_ + 1e-10, (max_y_ + min_y_) / 2.,
                                 max_y_ - 1e-10, max_y_,         max_y_ + 1e-10, max_y_ + 10.};
    std::vector<double> z_values{min_z_ - 10.,   min_z_ - 1e-10, min_z_,         min_z_ + 1e-10, (max_z_ + min_z_) / 2.,
                                 max_z_ - 1e-10, max_z_,         max_z_ + 1e-10, max_z_ + 10.};
    x_test_cases_ = x_values;
    y_test_cases_ = y_values;
    z_test_cases_ = z_values;
  }

  std::vector<int> GetTransgressedBoundaries(const api::GeoPositionT<drake::AutoDiffXd>& geo_position) {
    const int kXIndex = 0;
    const int kYIndex = 1;
    const int kZIndex = 2;

    std::vector<int> result{};
    const double x = geo_position.x().value();
    if (x < min_x_ || x > max_x_) result.push_back(kXIndex);
    const double y = geo_position.y().value();
    if (y < min_y_ || y > max_y_) result.push_back(kYIndex);
    const double z = geo_position.z().value();
    if (z < min_z_ || z > max_z_) result.push_back(kZIndex);

    return result;
  }

  std::unique_ptr<RoadGeometry> road_geometry_;
  const api::Segment* segment_{nullptr};
  const Lane* lane_{nullptr};

  const double length_{};
  const double lane_width_{};
  const double shoulder_width_{};
  const double maximum_height_{};

  const double min_x_{};
  const double max_x_{};
  const double min_y_{};
  const double max_y_{};
  const double min_z_{};
  const double max_z_{};

  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the segment r_min and r_max.
  const double kLinearTolerance = 1e-15;

  const double kAngularTolerance = std::numeric_limits<double>::epsilon();

  // For tests involving ToLanePosition().
  std::vector<double> x_test_cases_{};
  std::vector<double> y_test_cases_{};
  std::vector<double> z_test_cases_{};
};

/*
 Tests a dragway containing one lane. It is arranged as shown below in the world
 frame:

               x
               ^
      |<-------|------->|    segment r_max / r_min
      | |<-----|----->| |    lane    r_max / r_min
      -------------------    s = length_
      | |      ^      | |
      | |      :      | |
      | |      ^      | |
      | |      :      | |
  y <----------o---------->  s = 0
               |
               V
 */
TEST_F(MaliputDragwayLaneTest, SingleLane) {
  const int kNumLanes = 1;
  MakeDragway(kNumLanes);

  EXPECT_EQ(segment_->id(), api::SegmentId("Dragway_Segment_ID"));
  EXPECT_EQ(lane_->segment(), segment_);

  VerifyLaneCorrectness(lane_, kNumLanes);
  VerifyBranches(lane_, road_geometry_.get());
  EXPECT_TRUE(api::test::CheckIdIndexing(road_geometry_.get()));
}

/*
 Tests a dragway containing two lanes. The two lanes are arranged as shown below
 in the world frame:

                              x
                              ^
                              |
              |<-------|--------------------->|  lane 1 segment r_max / r_min
              |<---------------------|------->|  lane 0 segment r_max / r_min
              | |<-----|----->|<-----|----->| |  lane           r_max / r_min
              ----------------|----------------  s = length_
              | |      ^      |      ^      | |
              | |      :      |      :      | |
              | |      ^      |      ^      | |
              | |      :      |      :      | |  s = 0
      y <---------------------o------------------------------>
                    index 1   |    index 0
                              V
 */
TEST_F(MaliputDragwayLaneTest, TwoLaneDragway) {
  const int kNumLanes = 2;
  MakeDragway(kNumLanes);

  EXPECT_EQ(segment_->id(), api::SegmentId("Dragway_Segment_ID"));

  for (int i = 0; i < kNumLanes; ++i) {
    const api::Lane* lane = segment_->lane(i);
    ASSERT_NE(lane, nullptr);
    VerifyLaneCorrectness(lane, kNumLanes);
    VerifyBranches(lane, road_geometry_.get());
  }
  EXPECT_TRUE(api::test::CheckIdIndexing(road_geometry_.get()));
}

// Tests dragway::RoadGeometry::ToRoadPosition() using a two-lane dragway where
// the x,y projection of all geographic positions provided to it are in the
// road' segment region. This unit test also verifies that
// dragway::RoadGeometry::IsGeoPositionOnDragway() does not incorrectly return
// false.
TEST_F(MaliputDragwayLaneTest, TestToRoadPositionOnRoad) {
  MakeDragway(2 /* num lanes */);

  // Spot checks geographic positions on lane 0 and the right shoulder with a
  // focus on edge cases.
  for (double x = 0; x <= length_; x += length_ / 2) {
    for (double y = -lane_width_ - shoulder_width_; y <= 0; y += (lane_width_ + shoulder_width_) / 2) {
      for (double z = 0; z <= maximum_height_; z += maximum_height_ / 2.) {
        const api::RoadPositionResult result = road_geometry_->ToRoadPosition(api::GeoPosition(x, y, z));
        const api::Lane* expected_lane = road_geometry_->junction(0)->segment(0)->lane(0);
        EXPECT_TRUE(
            api::test::IsGeoPositionClose(result.nearest_position, api::GeoPosition(x, y, z), kLinearTolerance));
        EXPECT_DOUBLE_EQ(result.distance, 0);
        EXPECT_EQ(result.road_position.lane, expected_lane);
        EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                                   api::LanePosition(x, y + lane_width_ / 2, z), kLinearTolerance));
      }
    }
  }

  // Spot checks geographic positions on lane 1 and the left shoulder with a
  // focus on edge cases.
  for (double x = 0; x <= length_; x += length_ / 2) {
    for (double y = 0; y <= lane_width_ + shoulder_width_; y += (lane_width_ + shoulder_width_) / 2) {
      for (double z = 0; z <= maximum_height_; z += maximum_height_ / 2.) {
        const api::RoadPositionResult result = road_geometry_->ToRoadPosition(api::GeoPosition(x, y, z));
        const int lane_index = (y == 0 ? 0 : 1);
        const api::Lane* expected_lane = road_geometry_->junction(0)->segment(0)->lane(lane_index);
        EXPECT_TRUE(
            api::test::IsGeoPositionClose(result.nearest_position, api::GeoPosition(x, y, z), kLinearTolerance));
        EXPECT_DOUBLE_EQ(result.distance, 0);
        EXPECT_EQ(result.road_position.lane, expected_lane);
        if (y == 0) {
          EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                                     api::LanePosition(x, y + lane_width_ / 2, z), kLinearTolerance));
        } else {
          EXPECT_TRUE(api::test::IsLanePositionClose(result.road_position.pos,
                                                     api::LanePosition(x, y - lane_width_ / 2, z), kLinearTolerance));
        }
      }
    }
  }
}

// Tests dragway::RoadGeometry::ToRoadPosition() using a two-lane dragway where
// the x,y projection of all geographic positions provided to it may be outside
// of the road' segment region. This unit test also verifies that
// dragway::RoadGeometry::IsGeoPositionOnDragway() does not incorrectly return
// false.
TEST_F(MaliputDragwayLaneTest, TestToRoadPositionOffRoad) {
  MakeDragway(2 /* num lanes */);

  // Computes the bounds of the road' segment region.
  const double x_max = length_;
  const double x_min = 0;
  const double y_max = lane_width_ + shoulder_width_;
  const double y_min = -y_max;
  const double z_min = 0;
  const double z_max = maximum_height_;

  // Spot checks a region that is a superset of the road' segment bounds with
  // a focus on edge cases.
  const double test_x_min = x_min - 10;
  const double test_x_max = x_max + 10;
  const double test_y_min = y_min - 10;
  const double test_y_max = y_max + 10;
  const double test_z_min = z_min - 10;
  const double test_z_max = z_max + 10;

  // Defines the test case values for x and y. Points both far away and close to
  // the segment area are evaluated.
  const std::vector<double> x_test_cases{test_x_min, x_min - 1e-10, x_max + 1e-10, test_x_max};
  const std::vector<double> y_test_cases{test_y_min, y_min - 1e-10, y_max + 1e-10, test_y_max};
  const std::vector<double> z_test_cases{test_z_min, z_min - 1e-10, z_max + 1e-10, test_z_max};
  // TODO(maddog@tri.global)  Needs test cases where *one* coordinate is
  //                          out-of-bounds.  (Currently, only tests when
  //                          *all* coordinates are out-of-bounds.)
  for (const auto x : x_test_cases) {
    for (const auto y : y_test_cases) {
      for (const auto z : z_test_cases) {
        const api::RoadPositionResult result = road_geometry_->ToRoadPosition(api::GeoPosition(x, y, z));
        EXPECT_LE(result.nearest_position.x(), x_max);
        EXPECT_GE(result.nearest_position.x(), x_min);
        EXPECT_LE(result.nearest_position.y(), y_max);
        EXPECT_GE(result.nearest_position.y(), y_min);
        EXPECT_LE(result.nearest_position.z(), z_max);
        EXPECT_GE(result.nearest_position.z(), z_min);

        api::GeoPosition expected_nearest_position;
        expected_nearest_position.set_x(x);
        expected_nearest_position.set_y(y);
        expected_nearest_position.set_z(z);
        if (x < x_min) {
          expected_nearest_position.set_x(x_min);
        }
        if (x > x_max) {
          expected_nearest_position.set_x(x_max);
        }
        if (y < y_min) {
          expected_nearest_position.set_y(y_min);
        }
        if (y > y_max) {
          expected_nearest_position.set_y(y_max);
        }
        if (z < z_min) {
          expected_nearest_position.set_z(z_min);
        }
        if (z > z_max) {
          expected_nearest_position.set_z(z_max);
        }

        EXPECT_TRUE(
            api::test::IsGeoPositionClose(result.nearest_position, expected_nearest_position, kLinearTolerance));
        // TODO(maddog@tri.global)  Should test for explicit correct distance.
        EXPECT_LT(0, result.distance);
        const int expected_lane_index = (y > 0 ? 1 : 0);
        const Lane* expected_lane =
            dynamic_cast<const Lane*>(road_geometry_->junction(0)->segment(0)->lane(expected_lane_index));
        EXPECT_EQ(result.road_position.lane, expected_lane);
        EXPECT_TRUE(api::test::IsLanePositionClose(
            result.road_position.pos,
            api::LanePosition(expected_nearest_position.x(), expected_nearest_position.y() - expected_lane->y_offset(),
                              expected_nearest_position.z()),
            kLinearTolerance));
      }
    }
  }
}

// Tests dragway::Lane::ToLanePosition() using geographic positions whose
// projections onto the XY plane reside within the lane's region.
TEST_F(MaliputDragwayLaneTest, TestToLanePosition) {
  MakeDragway(1 /* num lanes */);
  PopulateGeoPositionTestCases();

  for (const double x : x_test_cases_) {
    for (const double y : y_test_cases_) {
      for (const double z : z_test_cases_) {
        const api::LanePositionResult result = lane_->ToLanePosition(api::GeoPosition(x, y, z));
        api::GeoPosition expected_nearest_position(x, y, z);
        if (x < min_x_) {
          expected_nearest_position.set_x(min_x_);
        }
        if (x > max_x_) {
          expected_nearest_position.set_x(max_x_);
        }
        if (y < min_y_) {
          expected_nearest_position.set_y(min_y_);
        }
        if (y > max_y_) {
          expected_nearest_position.set_y(max_y_);
        }
        if (z < min_z_) {
          expected_nearest_position.set_z(min_z_);
        }
        if (z > max_z_) {
          expected_nearest_position.set_z(max_z_);
        }

        EXPECT_TRUE(
            api::test::IsGeoPositionClose(result.nearest_position, expected_nearest_position, kLinearTolerance));
        // TODO(maddog@tri.global)  Should test for explicit correct distance.
        EXPECT_GE(result.distance, 0);
        EXPECT_TRUE(api::test::IsLanePositionClose(
            result.lane_position,
            api::LanePosition(expected_nearest_position.x(), expected_nearest_position.y() - lane_->y_offset(),
                              expected_nearest_position.z()),
            kLinearTolerance));
      }
    }
  }
}

// Tests that drake::AutoDiffXd returns values whose sizes and values match what is
// expected, and preserving any partial derivatives given to it.
TEST_F(MaliputDragwayLaneTest, TestToLanePositionAutoDiff) {
  MakeDragway(1 /* num lanes */);
  PopulateGeoPositionTestCases();

  for (const double x : x_test_cases_) {
    for (const double y : y_test_cases_) {
      for (const double z : z_test_cases_) {
        // Define the partial derivatives with respect to geo_position's
        // x-axis.  That is, seed geo_position with the partial derivatives
        // ∂x/∂x = 1, ∂y/∂x = 0., ∂z/∂x = 0, etc.
        drake::AutoDiffXd x_autodiff{x, drake::Vector3<double>(1., 0., 0.)};
        drake::AutoDiffXd y_autodiff{y, drake::Vector3<double>(0., 1., 0.)};
        drake::AutoDiffXd z_autodiff{z, drake::Vector3<double>(0., 0., 1.)};

        api::GeoPositionT<drake::AutoDiffXd> geo_position{x_autodiff, y_autodiff, z_autodiff};

        // Compute LanePositionT with and without the I/O arguments.
        const api::LanePositionResultT<drake::AutoDiffXd> result =
            lane_->ToLanePositionT<drake::AutoDiffXd>(geo_position);

        std::vector<drake::Vector3<double>> positional_derivatives{drake::Vector3<double>::Zero(),   // s
                                                                   drake::Vector3<double>::Zero(),   // r
                                                                   drake::Vector3<double>::Zero()};  // h
        for (int i{0}; i < 3; ++i) positional_derivatives[i](i) = 1.;

        if (result.distance == 0.) {
          // `geo_position` is within the interior or on the boundary.
          //
          // For the x-coordinate, expect the following derivatives for
          // lane_position within the interior: ∂s/∂x = 1, ∂r/∂x = 0., ∂h/∂x
          // = 0 (Dragway's s-r-h axis has the same orientation as the x-y-z
          // axis).  Verify that the derivatives of nearest position are the
          // same as for geo_position.
          for (int i{0}; i < 3; ++i) {
            EXPECT_TRUE(CompareMatrices(positional_derivatives[i], result.lane_position.srh()(i).derivatives()));
            EXPECT_TRUE(CompareMatrices(positional_derivatives[i], result.nearest_position.xyz()(i).derivatives()));
          }
          // Expect ∂/∂x(distance) = 0., as distance remains unchanged when
          // on the boundary or within the interior of the lane.
          EXPECT_TRUE(CompareMatrices(drake::Vector3<double>::Zero(), result.distance.derivatives()));
        } else {
          // `geo_position` is outside of the lane.  In this case, the
          // derivatives for distance and position depend on the region in
          // which `geo_position` resides.
          //
          // For instance, take some (x, y, z) such that min_x ≮ x ≮ max_x,
          // but min_y < y < max_y and min_z < z< max_z, then `geo_position`
          // lies closest to the s-facet.  We expect ∂(distance)/∂x = 1 when x
          // > max_x and -1 when x < min_x, but expect distance to be
          // invariant to y and z (i.e. ∂(distance)/∂y = ∂(distance)/∂z = 0).
          //
          // We further expect
          // ∂/∂x(lane_position.s()) = ∂/∂x(nearest_position.x()) = 0, but
          // ∂/∂y(lane_position.r()) = ∂/∂y(nearest_position.y()) = 1 and
          // ∂/∂z(lane_position.h()) = ∂/∂z(nearest_position.z()) = 1.
          //
          // The following diagram summarizes, slightly more generally, the
          // expected behavior with respect to x and y when z = 0.  Note that
          // we are depicting only the 2D subproblem for ease of
          // visualization.
          //
          //  y                                      (+)  (+)  (#)
          //  ^
          //  |          y = max_y  -----------------(o)--(o)  (-)
          //  |                     |                      |
          //  +----> x              |     Dragway    (o)  (o)  (-)
          //                        |                      |
          //                        ------------------------
          //                                               x = max_x
          //
          // Consider the points labeled above and let D = distance, LP =
          // lane_position, NP = nearest_position.  The following should hold:
          //
          // (+):  ∂/∂x(D) = 0, ∂/∂y(D) = 1
          //       ∂/∂x(LP.s()) = ∂/∂x(NP.x()) = 1
          //       ∂/∂y(LP.r()) = ∂/∂y(NP.y()) = 0
          // (-):  ∂/∂x(D) = 1, ∂/∂y(D) = 0
          //       ∂/∂x(LP.s()) = ∂/∂x(NP.x()) = 0
          //       ∂/∂y(LP.r()) = ∂/∂y(NP.y()) = 1
          // (#):  ∂/∂x(D) = √2/2, ∂/∂y(D) = √2/2  (assuming # is equilateral)
          //       ∂/∂x(LP.s()) = ∂/∂x(NP.x()) = 0
          //       ∂/∂y(LP.r()) = ∂/∂y(NP.y()) = 0
          // (o):  ∂/∂x(D) = 0, ∂/∂y(D) = 0
          //       ∂/∂x(LP.s()) = ∂/∂x(NP.x()) = 1
          //       ∂/∂y(LP.r()) = ∂/∂y(NP.y()) = 1
          //       (Note these are the distance = 0 cases)
          //
          // The cross-derivatives ∂/∂y(LP.s()), ∂/∂y(NP.s()), ∂/∂x(LP.r()),
          // ∂/∂x(NP.r()) should always be zero.  Similar principles apply
          // in cases where z ≠ 0.
          const drake::Vector3<double> derivative_candidates(
              (x < min_x_) ? (x - min_x_) / result.distance.value() : (x - max_x_) / result.distance.value(),
              (y < min_y_) ? (y - min_y_) / result.distance.value() : (y - max_y_) / result.distance.value(),
              (z < min_z_) ? (z - min_z_) / result.distance.value() : (z - max_z_) / result.distance.value());
          drake::Vector3<double> distance_derivatives = drake::Vector3<double>::Zero();

          for (int index : GetTransgressedBoundaries(geo_position)) {
            distance_derivatives(index) = derivative_candidates(index);
            positional_derivatives[index](index) = 0.;  // (entry was = 1).
          }
          EXPECT_TRUE(CompareMatrices(distance_derivatives, result.distance.derivatives(),
                                      1e-15 /* account for small numerical errors */));
          for (int i{0}; i < 3; ++i) {
            EXPECT_TRUE(CompareMatrices(positional_derivatives[i], result.lane_position.srh()(i).derivatives()));
            EXPECT_TRUE(CompareMatrices(positional_derivatives[i], result.nearest_position.xyz()(i).derivatives()));
          }
        }
      }
    }
  }
}

TEST_F(MaliputDragwayLaneTest, ThrowIfGeoPosHasMismatchedDerivatives) {
  MakeDragway(1 /* num lanes */);

  // Expect the function to throw when given a triple whose derivatives have
  // mismatched sizes.
  drake::AutoDiffXd s{1., drake::Vector3<double>(1., 2., 3.)};
  drake::AutoDiffXd r{5., drake::Vector2<double>(1., 2.)};
  drake::AutoDiffXd h{7., drake::Vector1<double>(1.)};
  api::LanePositionT<drake::AutoDiffXd> lane_position(s, r, h);

  EXPECT_THROW(lane_->ToGeoPositionT<drake::AutoDiffXd>(lane_position), maliput::common::assertion_error);
}

TEST_F(MaliputDragwayLaneTest, ThrowIfLanePosHasMismatchedDerivatives) {
  MakeDragway(1 /* num lanes */);

  // Expect the function to throw when given a triple whose derivatives have
  // mismatched sizes.
  drake::AutoDiffXd x{1., drake::Vector3<double>(1., 2., 3.)};
  drake::AutoDiffXd y{5., drake::Vector2<double>(1., 2.)};
  drake::AutoDiffXd z{7., drake::Vector1<double>(1.)};
  api::GeoPositionT<drake::AutoDiffXd> geo_position(x, y, z);

  EXPECT_THROW(lane_->ToLanePositionT<drake::AutoDiffXd>(geo_position), common::assertion_error);
}

TEST_F(MaliputDragwayLaneTest, ToLanePositionSymbolic1) {
  // We want to show that the result from ToLanePositionSymbolic() corresponds
  // with the one from ToLanePosition() when x, y, z are instantiated with the
  // same values used to construct the input to ToLanePosition().

  MakeDragway(1 /* num lanes */);
  const drake::symbolic::Variable x{"x"};
  const drake::symbolic::Variable y{"y"};
  const drake::symbolic::Variable z{"z"};
  const double v_x = 1.0;
  const double v_y = 2.0;
  const double v_z = -0.5;
  drake::symbolic::Environment env{{{x, v_x}, {y, v_y}, {z, v_z}}};

  const api::LanePositionResultT<drake::symbolic::Expression> symbolic_result{
      lane_->ToLanePositionT<drake::symbolic::Expression>(api::GeoPositionT<drake::symbolic::Expression>{x, y, z})};

  const api::LanePositionResult double_result{lane_->ToLanePosition(api::GeoPosition{v_x, v_y, v_z})};

  EXPECT_EQ(symbolic_result.lane_position.s().Evaluate(env), double_result.lane_position.s());
  EXPECT_EQ(symbolic_result.lane_position.r().Evaluate(env), double_result.lane_position.r());
  EXPECT_EQ(symbolic_result.lane_position.h().Evaluate(env), double_result.lane_position.h());

  EXPECT_EQ(symbolic_result.nearest_position.x().Evaluate(env), double_result.nearest_position.x());
  EXPECT_EQ(symbolic_result.nearest_position.y().Evaluate(env), double_result.nearest_position.y());
  EXPECT_EQ(symbolic_result.nearest_position.z().Evaluate(env), double_result.nearest_position.z());
  EXPECT_EQ(symbolic_result.distance.Evaluate(env), double_result.distance);

  // These values (1.0, 2.0, -0.5) lie outside the lane. Therefore, 1) we have a
  // positive distance and 2) the symbolic_gp evaluated with those values is
  // different from (1.0, 2.0, -0.5).
  EXPECT_GT(double_result.distance, 0.0);
  EXPECT_FALSE(symbolic_result.nearest_position.x().Evaluate(env) == v_x &&
               symbolic_result.nearest_position.y().Evaluate(env) == v_y &&
               symbolic_result.nearest_position.z().Evaluate(env) == v_z);
}

TEST_F(MaliputDragwayLaneTest, ToLanePositionSymbolic2) {
  // We want to show that the result from ToLanePositionSymbolic() corresponds
  // with the one from ToLanePosition() when x, y, z are instantiated with the
  // same values used to construct the input to ToLanePosition().

  MakeDragway(1 /* num lanes */);
  const drake::symbolic::Variable x{"x"};
  const drake::symbolic::Variable y{"y"};
  const drake::symbolic::Variable z{"z"};
  const drake::symbolic::Variable d{"distance"};
  const double v_x = 1.0;
  const double v_y = 2.0;
  const double v_z = 0.0;
  const double v_d = 0.0;
  drake::symbolic::Environment env{{{x, v_x}, {y, v_y}, {z, v_z}, {d, v_d}}};

  const api::LanePositionResultT<drake::symbolic::Expression> symbolic_result{
      lane_->ToLanePositionT<drake::symbolic::Expression>(api::GeoPositionT<drake::symbolic::Expression>{x, y, z})};

  const api::LanePositionResult double_result{lane_->ToLanePosition(api::GeoPosition{v_x, v_y, v_z})};

  EXPECT_EQ(symbolic_result.lane_position.s().Evaluate(env), double_result.lane_position.s());
  EXPECT_EQ(symbolic_result.lane_position.r().Evaluate(env), double_result.lane_position.r());
  EXPECT_EQ(symbolic_result.lane_position.h().Evaluate(env), double_result.lane_position.h());

  EXPECT_EQ(symbolic_result.nearest_position.x().Evaluate(env), double_result.nearest_position.x());
  EXPECT_EQ(symbolic_result.nearest_position.y().Evaluate(env), double_result.nearest_position.y());
  EXPECT_EQ(symbolic_result.nearest_position.z().Evaluate(env), double_result.nearest_position.z());
  EXPECT_EQ(symbolic_result.distance.Evaluate(env), double_result.distance);

  // These values (1.0, 2.0, 0.0) lie on the lane. Therefore we have 1) a zero
  // distance and 2) the nearest_position evaluated with those values is (1.0, 2.0,
  // 0.0).
  EXPECT_TRUE(symbolic_result.nearest_position.x().Evaluate(env) == v_x &&
              symbolic_result.nearest_position.y().Evaluate(env) == v_y &&
              symbolic_result.nearest_position.z().Evaluate(env) == v_z &&
              symbolic_result.distance.Evaluate(env) == v_d);
}

TEST_F(MaliputDragwayLaneTest, ToGeoPositionSymbolic) {
  // We want to show that the result from ToGeoPositionSymbolic() corresponds
  // with the one from ToGeoPosition() when s, r, h are instantiated with the
  // same values used to construct the input to ToGeoPosition().
  MakeDragway(1 /* num lanes */);

  const drake::symbolic::Variable s{"s"};
  const drake::symbolic::Variable r{"r"};
  const drake::symbolic::Variable h{"h"};
  const double v_s = 1.0;
  const double v_r = 2.0;
  const double v_h = -0.5;
  drake::symbolic::Environment env{{{s, v_s}, {r, v_r}, {h, v_h}}};

  // Computes symbolic gp.
  const api::GeoPositionT<drake::symbolic::Expression> symbolic_gp{
      lane_->ToGeoPositionT<drake::symbolic::Expression>(api::LanePositionT<drake::symbolic::Expression>{s, r, h})};

  // Computes gp (of double).
  const api::GeoPosition gp{lane_->ToGeoPosition(api::LanePosition{v_s, v_r, v_h})};

  EXPECT_EQ(symbolic_gp.x().Evaluate(env), gp.x());
  EXPECT_EQ(symbolic_gp.y().Evaluate(env), gp.y());
  EXPECT_EQ(symbolic_gp.z().Evaluate(env), gp.z());
}

// Holds RoadPositionResult expected arguments.
struct RoadPositionResultExpectation {
  api::LanePosition lane_position;
  api::GeoPosition nearest_position;
  double distance{};
};

// Test parameters and expected results for MaliputDragwayFindRoadPositionTest
struct FindRoadPositionsTestParameters {
  int dragway_num_lanes{};
  double radius{};
  api::GeoPosition inertial_position;
  std::map<api::LaneId, RoadPositionResultExpectation> expected_results;
};

// Operator overload for GTest log.
std::ostream& operator<<(std::ostream& os, const FindRoadPositionsTestParameters& test_parameters) {
  os << "{dragway_num_lanes: " << test_parameters.dragway_num_lanes << ", radius: " << test_parameters.radius
     << ", inertial_position: " << test_parameters.inertial_position << ", expected_results: {...}}";
  return os;
}

// To understand the characteristics of the geometry, consult the
// dragway::Segment and dragway::Lane detailed class overview docs.
class MaliputDragwayFindRoadPositionTest : public ::testing::TestWithParam<FindRoadPositionsTestParameters> {
 public:
  const double kLength = 100.;
  const double kLaneWidth = 6.;
  const double kShoulderWidth = 0.5;
  const double kMaximumHeight = 5.0;

  // The following linear tolerance was empirically derived on a 64-bit Ubuntu
  // system. It is necessary due to inaccuracies in floating point calculations
  // and different ways of computing the segment r_min and r_max.
  const double kLinearTolerance = 1e-15;

  const double kAngularTolerance = std::numeric_limits<double>::epsilon();

  static std::vector<FindRoadPositionsTestParameters> TestParameters();

  void MakeDragway(int num_lanes) {
    const api::RoadGeometryId road_geometry_id(std::to_string(num_lanes) + "LaneDragwayRoadGeometry");
    road_geometry_.reset(new RoadGeometry(road_geometry_id, num_lanes, kLength, kLaneWidth, kShoulderWidth,
                                          kMaximumHeight, kLinearTolerance, kAngularTolerance));
  }

 protected:
  void SetUp() override {
    const FindRoadPositionsTestParameters test_parameters = GetParam();
    dragway_num_lanes_ = test_parameters.dragway_num_lanes;
    radius_ = test_parameters.radius;
    inertial_position_ = test_parameters.inertial_position;
    expected_results_ = test_parameters.expected_results;
  }

  int dragway_num_lanes_{};
  double radius_{};
  api::GeoPosition inertial_position_;
  std::map<api::LaneId, RoadPositionResultExpectation> expected_results_;
  std::unique_ptr<RoadGeometry> road_geometry_;
};

std::vector<FindRoadPositionsTestParameters> MaliputDragwayFindRoadPositionTest::TestParameters() {
  return {
      // For the following set of test cases, an Inertial Frame position is placed
      // outside the road volume on purpose expecting the method to derive the
      // right LanePosition for each coordinate (s, r, and h), not only a mapping
      // to the ground.

      // { Single-lane Dragway tests cases.
      // Point is outside the RoadGeometry volume with zero radius, no possible
      // match.
      {1, 0., api::GeoPosition(10., 20., 30.), {}},
      // Point is outside the RoadGeometry volume with small radius, no possible
      // match.
      {1, 10., api::GeoPosition(10., 20., 30.), {}},
      // Point is outside the RoadGeometry volume with big enough radius to produce
      // a match.
      {
          1,
          30.,
          api::GeoPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3.5, 5.), api::GeoPosition(10., 3.5, 5.), 29.95413160150032}},
          },
      },
      // Same result as before but with a relatively big radius.
      {
          1,
          1000.,
          api::GeoPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3.5, 5.), api::GeoPosition(10., 3.5, 5.), 29.95413160150032}},
          },
      },
      // Biggest radius possible, the closest point to Inertial position is
      // expected for all lanes (just one).
      {
          1,
          std::numeric_limits<double>::infinity(),
          api::GeoPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 3.5, 5.), api::GeoPosition(10., 3.5, 5.), 29.95413160150032}},
          },
      },
      // } Single-lane Dragway tests cases.

      // { Multiple-lane Dragway test cases.
      // Point is outside the RoadGeometry volume with zero radius, no possible
      // match.
      {3, 0., api::GeoPosition(10., 20., 30.), {}},
      // Point is outside the RoadGeometry volume with small radius, no possible
      // match.
      {3, 10., api::GeoPosition(10., 20., 30.), {}},
      // Point is outside the RoadGeometry volume with big enough radius to
      // produce a match.
      {
          3,
          28.,
          api::GeoPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 15.5, 5.), api::GeoPosition(10., 9.5, 5.), 27.115493725912497}},
              {api::LaneId("Dragway_Lane_1"),
               {api::LanePosition(10., 9.5, 5.), api::GeoPosition(10., 9.5, 5.), 27.115493725912497}},
              {api::LaneId("Dragway_Lane_2"),
               {api::LanePosition(10., 3.5, 5.), api::GeoPosition(10., 9.5, 5.), 27.115493725912497}},
          },
      },
      // Point is within the RoadGeometry volume, specifically at the interface.
      {
          3,
          0.,
          api::GeoPosition(10., 9.5, 5.),
          {
              {api::LaneId("Dragway_Lane_0"), {api::LanePosition(10., 15.5, 5.), api::GeoPosition(10., 9.5, 5.), 0.}},
              {api::LaneId("Dragway_Lane_1"), {api::LanePosition(10., 9.5, 5.), api::GeoPosition(10., 9.5, 5.), 0.}},
              {api::LaneId("Dragway_Lane_2"), {api::LanePosition(10., 3.5, 5.), api::GeoPosition(10., 9.5, 5.), 0.}},
          },
      },
      // Point is outside the RoadGeometry volume with infinite radius, all lanes
      // are expected to produce a result.
      {
          3,
          std::numeric_limits<double>::infinity(),
          api::GeoPosition(10., 20., 30.),
          {
              {api::LaneId("Dragway_Lane_0"),
               {api::LanePosition(10., 15.5, 5.), api::GeoPosition(10., 9.5, 5.), 27.115493725912497}},
              {api::LaneId("Dragway_Lane_1"),
               {api::LanePosition(10., 9.5, 5.), api::GeoPosition(10., 9.5, 5.), 27.115493725912497}},
              {api::LaneId("Dragway_Lane_2"),
               {api::LanePosition(10., 3.5, 5.), api::GeoPosition(10., 9.5, 5.), 27.115493725912497}},
          },
      },
      // Point is within the RoadGeometry volume, with infinite radius, all lanes
      // are expected to produce a result.
      {
          3,
          std::numeric_limits<double>::infinity(),
          api::GeoPosition(10., 9.5, 5.),
          {
              {api::LaneId("Dragway_Lane_0"), {api::LanePosition(10., 15.5, 5.), api::GeoPosition(10., 9.5, 5.), 0.}},
              {api::LaneId("Dragway_Lane_1"), {api::LanePosition(10., 9.5, 5.), api::GeoPosition(10., 9.5, 5.), 0.}},
              {api::LaneId("Dragway_Lane_2"), {api::LanePosition(10., 3.5, 5.), api::GeoPosition(10., 9.5, 5.), 0.}},
          },
      },
      // } Multiple-lane Dragway test cases.
  };
}

// Evaluates RoadGeometry::FindRoadPositions() using a single-lane Dragway with
// different radii and Inertial positions.
TEST_P(MaliputDragwayFindRoadPositionTest, FindRoadPositionsWithOneLane) {
  MakeDragway(dragway_num_lanes_);

  std::vector<api::RoadPositionResult> road_position_results =
      road_geometry_->FindRoadPositions(inertial_position_, radius_);
  EXPECT_EQ(road_position_results.size(), expected_results_.size());

  // Evaluates RoadPosition matching.
  for (const api::RoadPositionResult& dut : road_position_results) {
    EXPECT_TRUE(expected_results_.find(dut.road_position.lane->id()) != expected_results_.end());
    EXPECT_TRUE(api::test::IsLanePositionClose(
        dut.road_position.pos, expected_results_[dut.road_position.lane->id()].lane_position, kLinearTolerance));
    EXPECT_TRUE(api::test::IsGeoPositionClose(
        dut.nearest_position, expected_results_[dut.road_position.lane->id()].nearest_position, kLinearTolerance));
    EXPECT_NEAR(dut.distance, expected_results_[dut.road_position.lane->id()].distance, kLinearTolerance);
  }
}

INSTANTIATE_TEST_CASE_P(MaliputDragwayFindLaneMappingsTestGroup, MaliputDragwayFindRoadPositionTest,
                        ::testing::ValuesIn(MaliputDragwayFindRoadPositionTest::TestParameters()));

}  // namespace
}  // namespace dragway
}  // namespace maliput
