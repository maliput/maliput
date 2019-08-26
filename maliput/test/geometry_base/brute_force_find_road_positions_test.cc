/* clang-format off to disable clang-format-includes */
#include "maliput/geometry_base/brute_force_find_road_positions_strategy.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <algorithm>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "maliput/test_utilities/mock_geometry.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/api/lane.h"

using ::testing::Expectation;
using ::testing::ExpectationSet;
using ::testing::Matcher;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;

namespace maliput {
namespace geometry_base {
namespace test {
namespace {

class BruteForceInterface {
 public:
  virtual std::vector<api::RoadPositionResult> BruteForceFindStrategy(const api::RoadGeometry* rg, const api::GeoPosition& geo_position, double radius) = 0;
};

class BruteForceWrapper : public BruteForceInterface {
 public:
  std::vector<api::RoadPositionResult> BruteForceFindStrategy(const api::RoadGeometry* rg, const api::GeoPosition& geo_position, double radius) override {
   return BruteForceFindRoadPositionsStrategy(rg, geo_position, radius);
  }
};

class BruteForceMocker : public BruteForceWrapper {
 public:
  MOCK_METHOD(std::vector<api::RoadPositionResult>, BruteForceFindStrategy, (const api::RoadGeometry* rg, const api::GeoPosition& geo_position, double radius));
};

class GeoPositionMatcher : public MatcherInterface<const api::GeoPosition&> {
 public:
  GeoPositionMatcher(const api::GeoPosition& geo_position, double tolerance)
      : geo_position_(geo_position), tolerance_(tolerance) {}

  bool MatchAndExplain(const api::GeoPosition& other, MatchResultListener*) const override {
   double delta{};

   delta = std::abs(geo_position_.x() - other.x());
   if (delta > tolerance_) return false;

   delta = std::abs(geo_position_.y() - other.y());
   if (delta > tolerance_) return false;

   delta = std::abs(geo_position_.z() - other.z());
   if (delta > tolerance_) return false;

   return true;
  }

  void DescribeTo(std::ostream* os) const override {
   *os << "is within tolerance: [" << tolerance_ << "] of all x, y, and z coordinates in: [" << geo_position_ << "]."; 
  }

 private:
  const api::GeoPosition geo_position_;
  const double tolerance_{};
};

Matcher<const api::GeoPosition&> Matches(const api::GeoPosition& geo_position, double tolerance) {
  return MakeMatcher(new GeoPositionMatcher(geo_position, tolerance));
}

class LaneMock final : public MockLane {
 public:
  explicit LaneMock(const api::LaneId& id) : MockLane(id) {}
  MOCK_METHOD(api::LanePosition, DoToLanePosition, (const api::GeoPosition&, api::GeoPosition*, double*));
 
 private: 
  api::LanePosition DoToLanePosition(const api::GeoPosition& geo_pos, api::GeoPosition* nearest_point, double* distance) const override { 
   return api::LanePosition(0, 0, 0);
  }
  double distance;
};

class RoadGeometryMock final : public MockRoadGeometry {
 public:
  explicit RoadGeometryMock(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length) : MockRoadGeometry(id, linear_tolerance, angular_tolerance, scale_length) {}
  void set_lanes(std::vector<LaneMock*> lanes) { lanes_.assign(lanes.begin(), lanes.end()); }
  std::vector<LaneMock*> get_lanes() { return lanes_; }
 
 private:
  std::vector<LaneMock*> lanes_;
};

std::unique_ptr<RoadGeometryMock> CreateFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length) {
 auto road_geometry = std::make_unique<RoadGeometryMock>(id, linear_tolerance, angular_tolerance, scale_length);
 
 auto lane0 = std::make_unique<LaneMock>(api::LaneId("lane0"));	
 auto lane1 = std::make_unique<LaneMock>(api::LaneId("lane1"));	
 auto lane2 = std::make_unique<LaneMock>(api::LaneId("lane2"));	
 
 std::vector<LaneMock*> lanes{ lane0.get(), lane1.get(), lane2.get() };

 road_geometry->set_lanes(lanes);

 auto segment0 = std::make_unique<MockSegment>(api::SegmentId("segment0"));
 auto segment1 = std::make_unique<MockSegment>(api::SegmentId("segment1"));
 
 segment0->AddLane(std::move(lane0));
 segment1->AddLane(std::move(lane1));
 segment1->AddLane(std::move(lane2));

 auto junction0 = std::make_unique<MockJunction>(api::JunctionId("junction0"));
 auto junction1 = std::make_unique<MockJunction>(api::JunctionId("junction1"));

 junction0->AddSegment(std::move(segment0)); 
 junction1->AddSegment(std::move(segment1)); 

 road_geometry->AddJunction(std::move(junction0));
 road_geometry->AddJunction(std::move(junction1));

 return road_geometry;
}

/*
class SegmentMock : public geometry_base::Segment {
 public:
  explicit SegmentMock(const api::SegmentId& id) {
   // TODO assign segment_ here
   //segment_ = std::make_unique<geometry_base::Segment>(id);
  }
 private:
  std::unique_ptr<Segment> segment_;
};

MockRoadGeometry CreateFullRoadGeometry(const api::RoadGeometryId& id, double linear_tolerance, double angular_tolerance, double scale_length) {

}
*/
GTEST_TEST(BruteForceTest, NullRoadGeometry) {
   auto local_bf = std::make_unique<BruteForceMocker>();
   BruteForceMocker* bf = local_bf.get();
   EXPECT_THROW(bf->BruteForceFindStrategy(nullptr, api::GeoPosition(0., 0., 0.), 0.), std::exception);
}

GTEST_TEST(BruteForceTest, NegativeRadius) {
   std::unique_ptr<MockRoadGeometry> rg = CreateFullRoadGeometry(api::RoadGeometryId("dut"), 1., 1., 1.);
   MockRoadGeometry* rg2 = rg.get();
   EXPECT_THROW(BruteForceFindRoadPositionsStrategy(rg2, api::GeoPosition(0., 0., 0.), -1.), std::exception);
}

GTEST_TEST(BruteForceTest, VerifyToLanePositiionArgs) {
  //auto local_rg = CreateFullRoadGeometry(api::RoadGeometryId("dut"), 1., 1., 1.);
 auto local_rg = std::make_unique<RoadGeometryMock>(api::RoadGeometryId("dut"), 1., 1., 1.);
 
 auto lane0 = std::make_unique<LaneMock>(api::LaneId("lane0"));	
 auto lane1 = std::make_unique<LaneMock>(api::LaneId("lane1"));	
 auto lane2 = std::make_unique<LaneMock>(api::LaneId("lane2"));	
 
 std::vector<LaneMock*> lanes{ lane0.get(), lane1.get(), lane2.get() };

 //road_geometry->set_lanes(lanes);

 auto segment0 = std::make_unique<MockSegment>(api::SegmentId("segment0"));
 auto segment1 = std::make_unique<MockSegment>(api::SegmentId("segment1"));
 
 

 segment0->AddLane(std::move(lane0));
 segment1->AddLane(std::move(lane1));
 segment1->AddLane(std::move(lane2));

 auto junction0 = std::make_unique<MockJunction>(api::JunctionId("junction0"));
 auto junction1 = std::make_unique<MockJunction>(api::JunctionId("junction1"));

 junction0->AddSegment(std::move(segment0)); 
 junction1->AddSegment(std::move(segment1)); 

 local_rg->AddJunction(std::move(junction0));
 local_rg->AddJunction(std::move(junction1));
  
  
  double distance{};
  api::GeoPosition nearest_position;
  RoadGeometryMock* rg = local_rg.get();
  
  ExpectationSet prebuild_expectations;
 
  auto local_bf = std::make_unique<BruteForceMocker>();
  BruteForceMocker* bf = local_bf.get();
  
  //std::vector<LaneMock*> lanes = rg->get_lanes();
/*
  for(auto lane : lanes){
    prebuild_expectations += EXPECT_CALL(
        *lane,
        DoToLanePosition(Matches(api::GeoPosition(1., 2., 3.), 0.01),
                         &nearest_position,
                         &distance));
  }
  */  
  EXPECT_CALL(
        *lanes[0],
        DoToLanePosition(Matches(api::GeoPosition(1., 2., 3.), 0.01),
                         &nearest_position,
                         &distance));
  EXPECT_CALL(*bf, BruteForceFindStrategy(rg, api::GeoPosition(1., 2., 3.), 1.));
  
  bf->BruteForceFindStrategy(rg, api::GeoPosition(1., 2., 3.), 1.);
}
/*
GTEST_TEST(BruteForceTest, NondefaultConstructionAndAccessors) {
  {
    const SRange dut(10., 50.);
    EXPECT_EQ(dut.s0(), 10.);
    EXPECT_EQ(dut.s1(), 50.);
  }
  // Inverted order is allowed and preserved:
  {
    SRange dut(79., 23.);
    EXPECT_EQ(dut.s0(), 79.);
    EXPECT_EQ(dut.s1(), 23.);
  }
}

GTEST_TEST(BruteForceTest, Setters) {
  SRange dut;
  dut.set_s0(26.);
  dut.set_s1(-90.);
  EXPECT_EQ(dut.s0(), 26.);
  EXPECT_EQ(dut.s1(), -90.);
}

GTEST_TEST(BruteForceTest, Copying) {
  const SRange source(12., 24.);
  const SRange dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(BruteForceTest, Assignment) {
  const SRange source(12., 24.);
  SRange dut;
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(BruteForceTest, ConstructionAndAccessors) {
  LaneSRange dut(LaneId("dut"), SRange(34., 0.));
  EXPECT_EQ(dut.lane_id(), LaneId("dut"));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.s_range(), SRange(34., 0.)));

  // Exercise convenient construction via initializer list for s_range.
  EXPECT_NO_THROW(LaneSRange(LaneId("dut"), {0., 50.}));
}

GTEST_TEST(BruteForceTest, Copying) {
  const LaneSRange source(LaneId("xxx"), SRange(20., 30.));
  const LaneSRange dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(LaneSRangeTest, Assignment) {
  const LaneSRange source(LaneId("xxx"), SRange(20., 30.));
  LaneSRange dut(LaneId("yyy"), SRange(40., 99.));  // e.g., "something else"
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}
*/
}  // namespace
}  // namespace test
}  // namespace geometry_base
}  // namespace maliput
