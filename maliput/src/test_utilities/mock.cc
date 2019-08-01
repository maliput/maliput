#include "maliput/test_utilities/mock.h"

#include <unordered_map>
#include <vector>

#include "maliput/api/branch_point.h"
#include "maliput/api/intersection.h"
#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/regions.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/api/segment.h"

#include "drake/common/drake_optional.h"

namespace maliput {
namespace api {
namespace test {
namespace {

using rules::DirectionUsageRule;
using rules::Phase;
using rules::PhaseRing;
using rules::RightOfWayRule;
using rules::SpeedLimitRule;
using rules::TrafficLight;

class MockIdIndex final : public RoadGeometry::IdIndex {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIdIndex);
  MockIdIndex() {}

 private:
  const Lane* DoGetLane(const LaneId&) const override { return nullptr; }

  const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const override {
    return lane_map_;
  }

  const Segment* DoGetSegment(const SegmentId&) const override {
    return nullptr;
  };
  const Junction* DoGetJunction(const JunctionId&) const override {
    return nullptr;
  };
  const BranchPoint* DoGetBranchPoint(const BranchPointId&) const override {
    return nullptr;
  }

  const std::unordered_map<LaneId, const Lane*> lane_map_;
};

class MockLane final : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockLane);
  MockLane() {}

 private:
  LaneId do_id() const override { return LaneId("mock"); };

  const Segment* do_segment() const override { return nullptr; };

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

  const BranchPoint* DoGetBranchPoint(const LaneEnd::Which) const override {
    return nullptr;
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

class MockOneLaneIdIndex final : public RoadGeometry::IdIndex {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockOneLaneIdIndex);
  MockOneLaneIdIndex() {}

 private:
  const Lane* DoGetLane(const LaneId&) const override { &mock_lane_; }

  const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const override {
    return lane_map_;
  }

  const Segment* DoGetSegment(const SegmentId&) const override {
    return nullptr;
  };
  const Junction* DoGetJunction(const JunctionId&) const override {
    return nullptr;
  };
  const BranchPoint* DoGetBranchPoint(const BranchPointId&) const override {
    return nullptr;
  }

  const MockLane mock_lane_;
  const std::unordered_map<LaneId, const Lane*> lane_map_{
      {LaneId("mock"), &mock_lane_}};
};

class MockRoadGeometry final : public RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadGeometry)
  MockRoadGeometry() {}

 private:
  RoadGeometryId do_id() const override { return RoadGeometryId("mock"); }
  int do_num_junctions() const override { return 1; }
  const Junction* do_junction(int) const override { return nullptr; };
  int do_num_branch_points() const override { return 1; }
  const BranchPoint* do_branch_point(int) const override { return nullptr; }
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
  MockIdIndex mock_id_index_;
};

class MockOneLaneRoadGeometry final : public RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockOneLaneRoadGeometry)
  MockOneLaneRoadGeometry() {}

 private:
  RoadGeometryId do_id() const override { return RoadGeometryId("mock"); }
  int do_num_junctions() const override { return 1; }
  const Junction* do_junction(int) const override { return nullptr; };
  int do_num_branch_points() const override { return 1; }
  const BranchPoint* do_branch_point(int) const override { return nullptr; }
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
  MockOneLaneIdIndex mock_id_index_;
};

class MockRoadRulebook final : public rules::RoadRulebook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadRulebook)
  MockRoadRulebook() {}

 private:
  QueryResults DoFindRules(const std::vector<rules::LaneSRange>&,
                           double) const override {
    return {{}, {}};
  }
  RightOfWayRule DoGetRule(const RightOfWayRule::Id&) const override {
    return Rule();
  }
  SpeedLimitRule DoGetRule(const SpeedLimitRule::Id&) const override {
    return SpeedLimitRule(rules::SpeedLimitRule::Id("some_id"),
                          CreateLaneSRange(),
                          rules::SpeedLimitRule::Severity::kStrict, 33., 77.);
  }

  DirectionUsageRule DoGetRule(const DirectionUsageRule::Id&) const override {
    return CreateDirectionUsageRule();
  }
};

class MockTrafficLightBook final : public rules::TrafficLightBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockTrafficLightBook)
  MockTrafficLightBook() {}

 private:
  drake::optional<TrafficLight> DoGetTrafficLight(
      const TrafficLight::Id&) const override {
    return drake::nullopt;
  }
};

class MockPhaseRingBook final : public rules::PhaseRingBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockPhaseRingBook)
  MockPhaseRingBook() {}

 private:
  std::vector<rules::PhaseRing::Id> DoGetPhaseRings() const override {
    return std::vector<rules::PhaseRing::Id>();
  }

  drake::optional<rules::PhaseRing> DoGetPhaseRing(
      const rules::PhaseRing::Id&) const override {
    return drake::nullopt;
  }

  drake::optional<rules::PhaseRing> DoFindPhaseRing(
      const rules::RightOfWayRule::Id&) const override {
    return drake::nullopt;
  }
};

class MockRuleStateProvider final : public rules::RuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRuleStateProvider)
  MockRuleStateProvider() {}

 private:
  drake::optional<RightOfWayResult> DoGetState(
      const RightOfWayRule::Id&) const override {
    return drake::nullopt;
  }
};

class MockPhaseProvider final : public rules::PhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockPhaseProvider)
  MockPhaseProvider() {}

 private:
  drake::optional<Result> DoGetPhase(
      const rules::PhaseRing::Id&) const override {
    return drake::nullopt;
  }
};

class MockIntersection final : public Intersection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIntersection)
  MockIntersection(const Intersection::Id& id, const rules::PhaseRing& ring)
      : Intersection(id, {}, ring) {}

 private:
  drake::optional<rules::PhaseProvider::Result> Phase() const override {
    return drake::nullopt;
  }

  void SetPhase(const api::rules::Phase::Id&) override {}
};

PhaseRing CreatePhaseRing() {
  return PhaseRing(PhaseRing::Id("mock"), {Phase(Phase::Id("mock"), {})});
}

class MockIntersectionBook final : public IntersectionBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIntersectionBook);
  MockIntersectionBook()
      : intersection_(Intersection::Id("Mock"), CreatePhaseRing()) {}

 private:
  std::vector<api::Intersection*> DoGetIntersections() {
    std::vector<api::Intersection*> result(1, &intersection_);
    return result;
  }

  api::Intersection* DoGetIntersection(const api::Intersection::Id& id) {
    if (id == intersection_.id()) {
      return &intersection_;
    }
    return nullptr;
  }

  MockIntersection intersection_;
};

}  // namespace

rules::LaneSRoute CreateLaneSRoute() {
  return rules::LaneSRoute({rules::LaneSRange(LaneId("a"), {0., 9.}),
                            rules::LaneSRange(LaneId("b"), {17., 12.})});
}

rules::LaneSRange CreateLaneSRange() {
  return rules::LaneSRange(LaneId("a"), {0., 9.});
}

RightOfWayRule::State::YieldGroup YieldGroup2() {
  return {RightOfWayRule::Id("other_rule_a"),
          RightOfWayRule::Id("other_rule_b")};
}

RightOfWayRule::State NoYieldState() {
  return RightOfWayRule::State(RightOfWayRule::State::Id("s1"),
                               RightOfWayRule::State::Type::kStop, {});
}

RightOfWayRule::State YieldState() {
  return RightOfWayRule::State(RightOfWayRule::State::Id("s2"),
                               RightOfWayRule::State::Type::kGo, YieldGroup2());
}

rules::RightOfWayRule::RelatedBulbGroups RelatedBulbGroups() {
  return rules::RightOfWayRule::RelatedBulbGroups{
      {rules::TrafficLight::Id("TrafficLightId"),
       {rules::BulbGroup::Id("BulbGroupId")}}};
}

RightOfWayRule Rule() {
  return RightOfWayRule(RightOfWayRule::Id("mock_id"), CreateLaneSRoute(),
                        RightOfWayRule::ZoneType::kStopExcluded,
                        {NoYieldState(), YieldState()}, RelatedBulbGroups());
}

DirectionUsageRule::State CreateDirectionUsageRuleState() {
  return DirectionUsageRule::State(
      DirectionUsageRule::State::Id("dur_state"),
      DirectionUsageRule::State::Type::kWithS,
      DirectionUsageRule::State::Severity::kStrict);
}

DirectionUsageRule CreateDirectionUsageRule() {
  return DirectionUsageRule(DirectionUsageRule::Id("dur_id"),
                            CreateLaneSRange(),
                            {CreateDirectionUsageRuleState()});
}

std::unique_ptr<RoadGeometry> CreateRoadGeometry() {
  return std::make_unique<MockRoadGeometry>();
}

std::unique_ptr<RoadGeometry> CreateOneLaneRoadGeometry() {
  return std::make_unique<MockOneLaneRoadGeometry>();
}

std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook() {
  return std::make_unique<MockRoadRulebook>();
}

std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook() {
  return std::make_unique<MockTrafficLightBook>();
}

std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook() {
  return std::make_unique<MockPhaseRingBook>();
}

std::unique_ptr<rules::RuleStateProvider> CreateRuleStateProvider() {
  return std::make_unique<MockRuleStateProvider>();
}

std::unique_ptr<rules::PhaseProvider> CreatePhaseProvider() {
  return std::make_unique<MockPhaseProvider>();
}

std::unique_ptr<IntersectionBook> CreateIntersectionBook() {
  return std::make_unique<MockIntersectionBook>();
}

}  // namespace test
}  // namespace api
}  // namespace maliput
