#include "maliput/api/road_network_validator.h"

#include <exception>

#include <gtest/gtest.h>

#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/lane_data.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/road_network.h"
#include "maliput/api/segment.h"
#include "maliput/common/assertion_error.h"
#include "maliput/common/maliput_throw.h"
#include "maliput/test_utilities/mock.h"

namespace maliput {
namespace api {
namespace {

using rules::DirectionUsageRule;
using rules::LaneSRange;
using rules::PhaseProvider;
using rules::PhaseRingBook;
using rules::RoadRulebook;
using rules::RuleStateProvider;
using rules::SpeedLimitRule;
using rules::SRange;
using rules::TrafficLightBook;

GTEST_TEST(RoadNetworkValidatorTest, RuleCoverageTest) {
  RoadNetwork road_network(test::CreateOneLaneRoadGeometry(), test::CreateRoadRulebook(),
                           test::CreateTrafficLightBook(), test::CreateIntersectionBook(), test::CreatePhaseRingBook(),
                           test::CreateRuleStateProvider(), test::CreatePhaseProvider());

  RoadNetworkValidatorOptions options{
      true /* check_direction_usage_rule_coverage */,
      false /* check_road_geometry_invariants */,
      false /* check_road_geometry_hierarchy */};
  EXPECT_THROW(ValidateRoadNetwork(road_network, options),
               maliput::common::assertion_error);

  options.check_direction_usage_rule_coverage = false;
  EXPECT_NO_THROW(ValidateRoadNetwork(road_network, options));
}


// Holds RoadGeometry build flag configuration.
struct RoadGeometryBuildFlags {
  bool add_junction{false};
  bool add_segment{false};
  bool add_lane{false};
  bool add_branchpoint{false};
  bool add_lane_end_set{false};
  bool expects_throw{false};
};

// Holds mock classes to build a RoadGeometry parametrically based on a
// RoadGeometryBuildFlags structure.
class RoadGeometryHierarchyTest :
    public ::testing::TestWithParam<RoadGeometryBuildFlags> {
 public:
  static std::vector<RoadGeometryBuildFlags> HierarchyTestParameters();

 protected:
  // { Mock classes.
  class MockLaneEndSet : public LaneEndSet {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockLaneEndSet)
    MockLaneEndSet() = default;

    // { Mock members.
    LaneEnd lane_end_;
    // } Mock members.
   private:
    int do_size() const override { return 1; }
    const LaneEnd& do_get(int index) const override {
      MALIPUT_THROW_UNLESS(index != 0);
      return lane_end_;
    }
  };

  class MockBranchPoint : public BranchPoint {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockBranchPoint)
    MockBranchPoint() = default;

    // { Mock members.
    RoadGeometry* road_geometry_{nullptr};
    std::unique_ptr<MockLaneEndSet> lane_end_set_a_;
    std::unique_ptr<MockLaneEndSet> lane_end_set_b_;
    // } Mock members.
   private:
    BranchPointId do_id() const override { return BranchPointId("mock"); }
    const RoadGeometry* do_road_geometry() const override {
      return road_geometry_;
    }
    const LaneEndSet* DoGetConfluentBranches(const LaneEnd&) const override {
      return nullptr;
    }
    const LaneEndSet* DoGetOngoingBranches(const LaneEnd&) const override {
      return nullptr;
    }
    drake::optional<LaneEnd> DoGetDefaultBranch(const LaneEnd&) const override {
      return drake::nullopt;
    }
    const LaneEndSet* DoGetASide() const override {
      return lane_end_set_a_.get();
    }
    const LaneEndSet* DoGetBSide() const override {
      return lane_end_set_b_.get();
    }
  };

  class MockLane final : public Lane {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockLane);
    MockLane() = default;

    // { Mock members.
    Segment* segment_{nullptr};
    BranchPoint* start_bp_{nullptr};
    BranchPoint* end_bp_{nullptr};
    // } Mock members.
   private:
    LaneId do_id() const override { return LaneId("mock"); };
    const Segment* do_segment() const override { return segment_; };
    int do_index() const override { return 0; };
    const Lane* do_to_left() const override { return nullptr; };
    const Lane* do_to_right() const override { return nullptr; };
    double do_length() const override { return 100; };
    RBounds do_lane_bounds(double) const override { return RBounds(-1, 1); };
    RBounds do_driveable_bounds(double) const override { return RBounds(-1, 1); };
    HBounds do_elevation_bounds(double, double) const override {
      return HBounds(0, 10);
    };
    GeoPosition DoToGeoPosition(const LanePosition&) const override {
      return GeoPosition(0, 0, 0);
    }
    LanePosition DoToLanePosition(const GeoPosition&, GeoPosition*,
                                  double*) const override {
      return LanePosition(0, 0, 0);
    }
    Rotation DoGetOrientation(const LanePosition&) const override {
      return Rotation();
    }
    LanePosition DoEvalMotionDerivatives(const LanePosition&,
                                         const IsoLaneVelocity&) const override {
      return LanePosition(0, 0, 0);
    }
    const BranchPoint* DoGetBranchPoint(const LaneEnd::Which end) const override {
      return end == LaneEnd::Which::kStart ? start_bp_ : end_bp_;
    };
    const LaneEndSet* DoGetConfluentBranches(
        const LaneEnd::Which) const override {
      return nullptr;
    };
    const LaneEndSet* DoGetOngoingBranches(const LaneEnd::Which) const override {
      return nullptr;
    };
    drake::optional<LaneEnd> DoGetDefaultBranch(
        const LaneEnd::Which) const override {
      return drake::nullopt;
    };
  };

  class MockSegment final : public Segment {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockSegment)
    MockSegment() = default;

    // { Mock members.
    Junction* junction_{nullptr};
    std::unique_ptr<MockLane> lane_;
    // } Mock members.
   private:
    SegmentId do_id() const override { return SegmentId("mock"); }
    const Junction* do_junction() const override { return junction_; }
    int do_num_lanes() const override { return lane_ != nullptr ? 1 : 0; }
    const Lane* do_lane(int index) const override {
      MALIPUT_THROW_UNLESS(index == 0);
      return lane_.get();
    }
  };

  class MockJunction final : public Junction {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockJunction)
    MockJunction() = default;

    // { Mock members.
    RoadGeometry* road_geometry_{nullptr};
    std::unique_ptr<MockSegment> segment_;
    // } Mock members.
   private:
    JunctionId do_id() const override { return JunctionId("mock"); }
    int do_num_segments() const override { return 1; }
    const Segment* do_segment(int i) const override {
      MALIPUT_THROW_UNLESS(i == 0);
      return segment_.get();
    }
    const RoadGeometry* do_road_geometry() const override {
      return road_geometry_;
    }
  };

  class MockIdIndex final : public RoadGeometry::IdIndex {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIdIndex);
    MockIdIndex() = default;

   private:
    const Lane* DoGetLane(const LaneId& lane_id) const override {
      return nullptr;
    }
    const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const override {
      return lane_map_;
    }
    const Segment* DoGetSegment(const SegmentId& segment_id) const override {
      return nullptr;
    };
    const Junction* DoGetJunction(const JunctionId& junction_id) const override {
      return nullptr;
    }
    const BranchPoint* DoGetBranchPoint(const BranchPointId&) const override {
      return nullptr;
    }

    const std::unordered_map<LaneId, const Lane*> lane_map_;
  };

  class MockRoadGeometry : public RoadGeometry {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadGeometry)
    MockRoadGeometry() = default;

    // { Mock members.
    std::unique_ptr<MockJunction> junction_;
    std::unique_ptr<MockBranchPoint> start_bp_;
    std::unique_ptr<MockBranchPoint> end_bp_;
    // } Mock members.
   private:
    RoadGeometryId do_id() const override { return RoadGeometryId("mock"); }
    int do_num_junctions() const override {
      return junction_ != nullptr ? 1 : 0;
    }
    const Junction* do_junction(int i) const override {
      MALIPUT_THROW_UNLESS(i == 0);
      return junction_.get();
    }
    int do_num_branch_points() const override {
      if (start_bp_ != nullptr && end_bp_ != nullptr) {
        return 2;
      } else if (start_bp_ == nullptr && end_bp_ == nullptr) {
        return 0;
      }
      return 1;
    }
    const BranchPoint* do_branch_point(int i) const override {
      MALIPUT_THROW_UNLESS(i == 0 || i == 1);
      return i == 0 ? start_bp_.get() : end_bp_.get();
    }
    const IdIndex& DoById() const override { return mock_id_index_; }
    RoadPosition DoToRoadPosition(const GeoPosition&, const RoadPosition*,
                                  GeoPosition*, double*) const override {
      return RoadPosition();
    }
    std::vector<api::RoadPositionResult> DoFindRoadPositions(
        const GeoPosition&, double) const override {
      return {{RoadPosition(), GeoPosition(), 0.}};
    }
    double do_linear_tolerance() const override { return 0; }
    double do_angular_tolerance() const override { return 0; }
    double do_scale_length() const override { return 0; }

    const MockIdIndex mock_id_index_;
  };
  // } Mock classes.

  // Builds a RoadGeometry parametrically based on `build_flags_` configuration.
  //
  // When `build_flags_.add_junction` is true, an empty Junction is added; and
  // if `build_flags_.add_segment` is true, an empty Segment is added to the
  // Junction; and  if `build_flags_.add_lane` is true, an empty Lane is added
  // to the Segment.
  // When `build_flags_.add_branchpoint` is true, two BranchPoints are added to
  // the RoadGeometry.
  // When all items in `build_flags_` are true, respective LaneEndSets are
  // created to link Lane with BranchPoints.
  //
  // @returns A MockRoadGeometry pointer.
  std::unique_ptr<const RoadGeometry> BuildMockRoadGeometry() const {
    auto rg = std::make_unique<MockRoadGeometry>();

    if (build_flags_.add_branchpoint) {
      rg->start_bp_= std::make_unique<MockBranchPoint>();
      rg->start_bp_->road_geometry_ = rg.get();

      rg->end_bp_= std::make_unique<MockBranchPoint>();
      rg->end_bp_->road_geometry_ = rg.get();
    }

    if (build_flags_.add_junction) {
      rg->junction_ = std::make_unique<MockJunction>();
      rg->junction_->road_geometry_ = rg.get();

      if (build_flags_.add_segment) {
        rg->junction_->segment_ = std::make_unique<MockSegment>();
        rg->junction_->segment_->junction_ = rg->junction_.get();

        if (build_flags_.add_lane) {
          rg->junction_->segment_->lane_ = std::make_unique<MockLane>();
          rg->junction_->segment_->lane_->segment_ =
              rg->junction_->segment_.get();

          if (build_flags_.add_branchpoint && build_flags_.add_lane_end_set) {
            rg->start_bp_->lane_end_set_a_ = std::make_unique<MockLaneEndSet>();
            rg->start_bp_->lane_end_set_a_->lane_end_ = LaneEnd(
                rg->junction_->segment_->lane_.get(), LaneEnd::Which::kStart);
            rg->start_bp_->lane_end_set_b_ = std::make_unique<MockLaneEndSet>();

            rg->end_bp_->lane_end_set_a_ = std::make_unique<MockLaneEndSet>();
            rg->end_bp_->lane_end_set_a_->lane_end_ = LaneEnd(
                rg->junction_->segment_->lane_.get(), LaneEnd::Which::kFinish);
            rg->end_bp_->lane_end_set_b_ = std::make_unique<MockLaneEndSet>();

            rg->junction_->segment_->lane_->start_bp_ = rg->start_bp_.get();
            rg->junction_->segment_->lane_->end_bp_ = rg->end_bp_.get();
          }
        }
      }
    }
    return std::move(rg);
  }


  void SetUp() override {
    build_flags_ = GetParam();
  }

  RoadGeometryBuildFlags build_flags_;
};

std::vector<RoadGeometryBuildFlags>
    RoadGeometryHierarchyTest::HierarchyTestParameters() {
  return {
    // Throws because of missing Junction.
    RoadGeometryBuildFlags{false, false, false, false, false, true},
    // Throws because of missing Segment in Junction.
    RoadGeometryBuildFlags{true, false, false, false, false, true},
    // Throws because of missing Lane in Segment.
    RoadGeometryBuildFlags{true, true, false, false, false, true},
    // Throws because of missing BranchPoint.
    RoadGeometryBuildFlags{true, true, true, false, false, true},
    // Throws because of missing LaneEndSet in BranchPoint.
    RoadGeometryBuildFlags{true, true, true, true, false, true},
    // Does not throw, complete RoadGeometry.
    RoadGeometryBuildFlags{true, true, true, true, true, false},
  };
}

TEST_P(RoadGeometryHierarchyTest, HierarchyTestThrows) {
  RoadNetwork road_network(
      BuildMockRoadGeometry(), test::CreateRoadRulebook(),
      test::CreateTrafficLightBook(), test::CreateIntersectionBook(),
      test::CreatePhaseRingBook(), test::CreateRuleStateProvider(),
      test::CreatePhaseProvider());

  const RoadNetworkValidatorOptions options{
      false /* check_direction_usage_rule_coverage */,
      false /* check_road_geometry_invariants */,
      true /* check_road_geometry_hierarchy */};

  if (build_flags_.expects_throw) {
    EXPECT_THROW({ ValidateRoadNetwork(road_network, options); },
                 maliput::common::assertion_error);
  } else {
    ValidateRoadNetwork(road_network, options);
    EXPECT_NO_THROW({ ValidateRoadNetwork(road_network, options); });
  }

}

INSTANTIATE_TEST_CASE_P(
    RoadGeometryHierarchyTestGroup, RoadGeometryHierarchyTest,
    ::testing::ValuesIn(RoadGeometryHierarchyTest::HierarchyTestParameters()));


}  // namespace
}  // namespace api
}  // namespace maliput
