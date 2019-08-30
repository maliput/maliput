#include "maliput/test_utilities/mock.h"

#include <unordered_map>
#include <utility>
#include <vector>

#include "maliput/api/branch_point.h"
#include "maliput/api/intersection.h"
#include "maliput/api/junction.h"
#include "maliput/api/lane.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/api/segment.h"

#include "drake/common/drake_optional.h"

namespace maliput {
namespace api {
namespace test {
namespace {

using rules::Bulb;
using rules::BulbGroup;
using rules::DirectionUsageRule;
using rules::DiscreteValueRule;
using rules::Phase;
using rules::PhaseRing;
using rules::RangeValueRule;
using rules::RightOfWayRule;
using rules::Rule;
using rules::SpeedLimitRule;
using rules::TrafficLight;

class MockLaneEndSet final : public LaneEndSet {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockLaneEndSet)
  MockLaneEndSet() = default;
  void set_lane_end(const LaneEnd& lane_end) { lane_end_ = lane_end; }

 private:
  int do_size() const override { return 1; }
  const LaneEnd& do_get(int index) const override {
    MALIPUT_THROW_UNLESS(index != 0);
    return lane_end_;
  }

  LaneEnd lane_end_;
};

class MockBranchPoint final : public BranchPoint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockBranchPoint)
  explicit MockBranchPoint(const BranchPointId& id) : BranchPoint(), id_(id) {}
  void set_road_geometry(RoadGeometry* road_geometry) { road_geometry_ = road_geometry; }
  void set_lane_end_set_a(std::unique_ptr<MockLaneEndSet> lane_end_set_a) {
    lane_end_set_a_ = std::move(lane_end_set_a);
  }
  void set_lane_end_set_b(std::unique_ptr<MockLaneEndSet> lane_end_set_b) {
    lane_end_set_b_ = std::move(lane_end_set_b);
  }

 private:
  BranchPointId do_id() const override { return id_; }
  const RoadGeometry* do_road_geometry() const override { return road_geometry_; }
  const LaneEndSet* DoGetConfluentBranches(const LaneEnd&) const override { return nullptr; }
  const LaneEndSet* DoGetOngoingBranches(const LaneEnd&) const override { return nullptr; }
  drake::optional<LaneEnd> DoGetDefaultBranch(const LaneEnd&) const override { return drake::nullopt; }
  const LaneEndSet* DoGetASide() const override { return lane_end_set_a_.get(); }
  const LaneEndSet* DoGetBSide() const override { return lane_end_set_b_.get(); }

  BranchPointId id_;
  RoadGeometry* road_geometry_{};
  std::unique_ptr<MockLaneEndSet> lane_end_set_a_;
  std::unique_ptr<MockLaneEndSet> lane_end_set_b_;
};

class MockLane final : public Lane {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockLane);
  MockLane(const LaneId& id) : Lane(), id_(id) {}
  void set_segment(Segment* segment) { segment_ = segment; }
  void set_start_bp(BranchPoint* start_bp) { start_bp_ = start_bp; }
  void set_end_bp(BranchPoint* end_bp) { end_bp_ = end_bp; }

 private:
  LaneId do_id() const override { return id_; };
  const Segment* do_segment() const override { return segment_; };
  int do_index() const override { return 0; };
  const Lane* do_to_left() const override { return nullptr; };
  const Lane* do_to_right() const override { return nullptr; };
  double do_length() const override { return 100; };
  RBounds do_lane_bounds(double) const override { return RBounds(-1, 1); };
  RBounds do_driveable_bounds(double) const override { return RBounds(-1, 1); };
  HBounds do_elevation_bounds(double, double) const override { return HBounds(0, 10); };
  GeoPosition DoToGeoPosition(const LanePosition&) const override { return GeoPosition(0, 0, 0); }
  LanePosition DoToLanePosition(const GeoPosition&, GeoPosition*, double*) const override {
    return LanePosition(0, 0, 0);
  }
  Rotation DoGetOrientation(const LanePosition&) const override { return Rotation(); }
  LanePosition DoEvalMotionDerivatives(const LanePosition&, const IsoLaneVelocity&) const override {
    return LanePosition(0, 0, 0);
  }
  const BranchPoint* DoGetBranchPoint(const LaneEnd::Which end) const override {
    return end == LaneEnd::Which::kStart ? start_bp_ : end_bp_;
  };
  const LaneEndSet* DoGetConfluentBranches(const LaneEnd::Which) const override { return nullptr; }
  const LaneEndSet* DoGetOngoingBranches(const LaneEnd::Which) const override { return nullptr; }
  drake::optional<LaneEnd> DoGetDefaultBranch(const LaneEnd::Which) const override { return drake::nullopt; }

  LaneId id_;
  Segment* segment_{};
  BranchPoint* start_bp_{};
  BranchPoint* end_bp_{};
};

class MockSegment final : public Segment {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockSegment)
  MockSegment(const SegmentId& id) : Segment(), id_(id) {}
  void set_junction(Junction* junction) { junction_ = junction; }
  void set_lane(std::unique_ptr<MockLane> lane) { lane_ = std::move(lane); }

 private:
  SegmentId do_id() const override { return id_; }
  const Junction* do_junction() const override { return junction_; }
  int do_num_lanes() const override { return lane_ != nullptr ? 1 : 0; }
  const Lane* do_lane(int index) const override {
    MALIPUT_THROW_UNLESS(index == 0);
    return lane_.get();
  }

  SegmentId id_;
  Junction* junction_{};
  std::unique_ptr<MockLane> lane_;
};

class MockJunction final : public Junction {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockJunction)
  MockJunction(const JunctionId& id) : Junction(), id_(id) {}
  void set_road_geometry(RoadGeometry* road_geometry) { road_geometry_ = road_geometry; }
  void set_segment(std::unique_ptr<MockSegment> segment) { segment_ = std::move(segment); }

 private:
  JunctionId do_id() const override { return JunctionId("mock"); }
  int do_num_segments() const override { return 1; }
  const Segment* do_segment(int i) const override {
    MALIPUT_THROW_UNLESS(i == 0);
    return segment_.get();
  }
  const RoadGeometry* do_road_geometry() const override { return road_geometry_; }

  JunctionId id_;
  RoadGeometry* road_geometry_{};
  std::unique_ptr<MockSegment> segment_;
};

class MockIdIndex final : public RoadGeometry::IdIndex {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIdIndex);
  MockIdIndex() = default;

 private:
  const Lane* DoGetLane(const LaneId& lane_id) const override { return nullptr; }
  const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const override { return lane_map_; }
  const Segment* DoGetSegment(const SegmentId& segment_id) const override { return nullptr; }
  const Junction* DoGetJunction(const JunctionId& junction_id) const override { return nullptr; }
  const BranchPoint* DoGetBranchPoint(const BranchPointId&) const override { return nullptr; }

  const std::unordered_map<LaneId, const Lane*> lane_map_;
};

class MockRoadGeometry : public RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadGeometry)
  MockRoadGeometry(const RoadGeometryId& id) : id_(id) {}

  void set_junction(std::unique_ptr<MockJunction> junction) { junction_ = std::move(junction); }
  void set_start_bp(std::unique_ptr<MockBranchPoint> start_bp) { start_bp_ = std::move(start_bp); }
  void set_end_bp(std::unique_ptr<MockBranchPoint> end_bp) { end_bp_ = std::move(end_bp); }
  MockBranchPoint* start_bp() { return start_bp_.get(); }
  MockBranchPoint* end_bp() { return end_bp_.get(); }

 private:
  RoadGeometryId do_id() const override { return id_; }
  int do_num_junctions() const override { return junction_ != nullptr ? 1 : 0; }
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
  RoadPosition DoToRoadPosition(const GeoPosition&, const RoadPosition*, GeoPosition*, double*) const override {
    return RoadPosition();
  }
  std::vector<api::RoadPositionResult> DoFindRoadPositions(const GeoPosition&, double) const override {
    return {{RoadPosition(), GeoPosition(), 0.}};
  }
  double do_linear_tolerance() const override { return 0; }
  double do_angular_tolerance() const override { return 0; }
  double do_scale_length() const override { return 0; }

  const MockIdIndex mock_id_index_;
  RoadGeometryId id_;
  std::unique_ptr<MockJunction> junction_;
  std::unique_ptr<MockBranchPoint> start_bp_;
  std::unique_ptr<MockBranchPoint> end_bp_;
};

class MockOneLaneIdIndex final : public RoadGeometry::IdIndex {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockOneLaneIdIndex);
  MockOneLaneIdIndex() : RoadGeometry::IdIndex() {}

 private:
  const Lane* DoGetLane(const LaneId&) const override { &mock_lane_; }

  const std::unordered_map<LaneId, const Lane*>& DoGetLanes() const override { return lane_map_; }

  const Segment* DoGetSegment(const SegmentId&) const override { return nullptr; };
  const Junction* DoGetJunction(const JunctionId&) const override { return nullptr; };
  const BranchPoint* DoGetBranchPoint(const BranchPointId&) const override { return nullptr; }

  const MockLane mock_lane_{LaneId{"mock"}};
  const std::unordered_map<LaneId, const Lane*> lane_map_{{LaneId("mock"), &mock_lane_}};
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
  RoadPosition DoToRoadPosition(const GeoPosition&, const RoadPosition*, GeoPosition*, double*) const override {
    return RoadPosition();
  }
  std::vector<api::RoadPositionResult> DoFindRoadPositions(const GeoPosition&, double) const override {
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
  void set_right_of_way(const RightOfWayRule& rule) { right_of_way_rule_ = rule; }
  void set_direction_usage(const DirectionUsageRule& rule) { direction_usage_rule_ = rule; }
  void set_speed_limit(const SpeedLimitRule& rule) { speed_limit_rule_ = rule; }
  void set_discrete_value_rule(const DiscreteValueRule& rule) { discrete_value_rule_ = rule; }
  void set_range_value_rule(const RangeValueRule& rule) { range_value_rule_ = rule; }

 private:
  QueryResults DoFindRules(const std::vector<LaneSRange>&, double) const override { return DoRules(); }
  QueryResults DoRules() const override {
    QueryResults result;
    if (right_of_way_rule_.has_value()) {
      result.right_of_way.emplace(right_of_way_rule_->id(), *right_of_way_rule_);
    }
    if (direction_usage_rule_.has_value()) {
      result.direction_usage.emplace(direction_usage_rule_->id(), *direction_usage_rule_);
    }
    if (speed_limit_rule_.has_value()) {
      result.speed_limit.emplace(speed_limit_rule_->id(), *speed_limit_rule_);
    }
    if (discrete_value_rule_.has_value()) {
      result.discrete_value_rules.emplace(discrete_value_rule_->id(), *discrete_value_rule_);
    }
    if (range_value_rule_.has_value()) {
      result.range_value_rules.emplace(range_value_rule_->id(), *range_value_rule_);
    }
    return result;
  }
  RightOfWayRule DoGetRule(const RightOfWayRule::Id&) const override { return *right_of_way_rule_; }
  SpeedLimitRule DoGetRule(const SpeedLimitRule::Id&) const override { return *speed_limit_rule_; }
  DirectionUsageRule DoGetRule(const DirectionUsageRule::Id&) const override { return *direction_usage_rule_; }
  DiscreteValueRule DoGetDiscreteValueRule(const Rule::Id& id) const override { return *discrete_value_rule_; }
  RangeValueRule DoGetRangeValueRule(const Rule::Id& id) const override { return *range_value_rule_; }

  drake::optional<RightOfWayRule> right_of_way_rule_{};
  drake::optional<DirectionUsageRule> direction_usage_rule_{};
  drake::optional<SpeedLimitRule> speed_limit_rule_{};
  drake::optional<DiscreteValueRule> discrete_value_rule_{};
  drake::optional<RangeValueRule> range_value_rule_{};
};

class MockTrafficLightBook final : public rules::TrafficLightBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockTrafficLightBook)
  MockTrafficLightBook() = default;
  void set_traffic_light(const TrafficLight& traffic_light) { traffic_light_ = traffic_light; }

 private:
  drake::optional<TrafficLight> DoGetTrafficLight(const TrafficLight::Id& id) const override {
    if (traffic_light_.has_value() && traffic_light_->id() == id) {
      return traffic_light_;
    }
    return {drake::nullopt};
  }
  std::vector<TrafficLight> DoTrafficLights() const override {
    if (traffic_light_.has_value()) {
      return {*traffic_light_};
    }
    return {};
  }

  drake::optional<TrafficLight> traffic_light_{};
};

class MockPhaseRingBook final : public rules::PhaseRingBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockPhaseRingBook)
  MockPhaseRingBook() {}

 private:
  std::vector<rules::PhaseRing::Id> DoGetPhaseRings() const override { return std::vector<rules::PhaseRing::Id>(); }

  drake::optional<rules::PhaseRing> DoGetPhaseRing(const rules::PhaseRing::Id&) const override {
    return drake::nullopt;
  }

  drake::optional<rules::PhaseRing> DoFindPhaseRing(const rules::RightOfWayRule::Id&) const override {
    return drake::nullopt;
  }
};

class MockRuleStateProvider final : public rules::RuleStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRuleStateProvider)
  MockRuleStateProvider() {}

 private:
  drake::optional<RightOfWayResult> DoGetState(const RightOfWayRule::Id&) const override { return drake::nullopt; }
};

class MockPhaseProvider final : public rules::PhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockPhaseProvider)
  MockPhaseProvider() {}

 private:
  drake::optional<Result> DoGetPhase(const rules::PhaseRing::Id&) const override { return drake::nullopt; }
};

class MockIntersection final : public Intersection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIntersection)
  MockIntersection(const Intersection::Id& id, const rules::PhaseRing& ring) : Intersection(id, {}, ring) {}

 private:
  drake::optional<rules::PhaseProvider::Result> Phase() const override { return drake::nullopt; }

  void SetPhase(const api::rules::Phase::Id&, const drake::optional<api::rules::Phase::Id>& next_phase = drake::nullopt,
                const drake::optional<double>& duration_until = drake::nullopt) override {}
};

PhaseRing CreatePhaseRing() { return PhaseRing(PhaseRing::Id("mock"), {Phase(Phase::Id("mock"), {})}); }

class MockIntersectionBook final : public IntersectionBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIntersectionBook);
  MockIntersectionBook() : intersection_(Intersection::Id("Mock"), CreatePhaseRing()) {}

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

LaneSRoute CreateLaneSRoute() {
  return LaneSRoute({LaneSRange(LaneId("a"), {0., 9.}), LaneSRange(LaneId("b"), {17., 12.})});
}

LaneSRange CreateLaneSRange() { return LaneSRange(LaneId("a"), {0., 9.}); }

RightOfWayRule::State::YieldGroup YieldGroup2() {
  return {RightOfWayRule::Id("other_rule_a"), RightOfWayRule::Id("other_rule_b")};
}

RightOfWayRule::State NoYieldState() {
  return RightOfWayRule::State(RightOfWayRule::State::Id("s1"), RightOfWayRule::State::Type::kStop, {});
}

RightOfWayRule::State YieldState() {
  return RightOfWayRule::State(RightOfWayRule::State::Id("s2"), RightOfWayRule::State::Type::kGo, YieldGroup2());
}

RightOfWayRule::RelatedBulbGroups RelatedBulbGroups() {
  return rules::RightOfWayRule::RelatedBulbGroups{
      {rules::TrafficLight::Id("TrafficLightId"), {rules::BulbGroup::Id("BulbGroupId")}}};
}

RightOfWayRule CreateRightOfWayRule() { return CreateRightOfWayRule(RightOfWayBuildFlags{}); }

RightOfWayRule CreateRightOfWayRule(const RightOfWayBuildFlags& build_flags) {
  return RightOfWayRule(
      RightOfWayRule::Id("mock_id"), CreateLaneSRoute(), RightOfWayRule::ZoneType::kStopExcluded,
      {NoYieldState(), YieldState()},
      build_flags.add_related_bulb_groups ? RelatedBulbGroups() : RightOfWayRule::RelatedBulbGroups{});
}

SpeedLimitRule CreateSpeedLimitRule() {
  return SpeedLimitRule(SpeedLimitRule::Id("some_id"), CreateLaneSRange(), SpeedLimitRule::Severity::kStrict, 33., 77.);
}

DirectionUsageRule::State CreateDirectionUsageRuleState() {
  return DirectionUsageRule::State(DirectionUsageRule::State::Id("dur_state"), DirectionUsageRule::State::Type::kWithS,
                                   DirectionUsageRule::State::Severity::kStrict);
}

DirectionUsageRule CreateDirectionUsageRule() {
  return DirectionUsageRule(DirectionUsageRule::Id("dur_id"), CreateLaneSRange(), {CreateDirectionUsageRuleState()});
}

DiscreteValueRule CreateDiscreteValueRule() {
  return DiscreteValueRule(Rule::Id("dvrt/dvr_id"), Rule::TypeId("dvrt"), CreateLaneSRoute(),
                           {rules::MakeDiscreteValue(rules::Rule::State::kStrict, {} /* related rules */, "value1"),
                            rules::MakeDiscreteValue(rules::Rule::State::kStrict, {} /* related rules */, "value2")});
}

RangeValueRule::Range CreateRange() {
  return rules::MakeRange(0 /* severity */, {} /* related rules */, "description", 123. /* min */, 456. /* max */);
}

RangeValueRule CreateRangeValueRule() {
  return RangeValueRule(Rule::Id("rvrt/rvr_id"), Rule::TypeId("dvrt"), CreateLaneSRoute(), {CreateRange()});
}

std::unique_ptr<RoadGeometry> CreateRoadGeometry() {
  return std::make_unique<MockRoadGeometry>(RoadGeometryId("mock"));
}

std::unique_ptr<RoadGeometry> CreateRoadGeometry(const RoadGeometryBuildFlags& build_flags) {
  auto rg = std::make_unique<MockRoadGeometry>(RoadGeometryId("mock"));
  if (build_flags.add_branchpoint) {
    auto start_bp = std::make_unique<MockBranchPoint>(BranchPointId("mock_start"));
    start_bp->set_road_geometry(rg.get());
    rg->set_start_bp(std::move(start_bp));
    auto end_bp = std::make_unique<MockBranchPoint>(BranchPointId("mock_end"));
    end_bp->set_road_geometry(rg.get());
    rg->set_end_bp(std::move(end_bp));
  }
  if (build_flags.add_junction) {
    auto junction = std::make_unique<MockJunction>(JunctionId("mock"));
    junction->set_road_geometry(rg.get());
    if (build_flags.add_segment) {
      auto segment = std::make_unique<MockSegment>(SegmentId("mock"));
      segment->set_junction(junction.get());
      if (build_flags.add_lane) {
        auto lane = std::make_unique<MockLane>(LaneId("mock"));
        lane->set_segment(segment.get());
        if (build_flags.add_branchpoint && build_flags.add_lane_end_set) {
          auto lane_end_set_start = std::make_unique<MockLaneEndSet>();
          lane_end_set_start->set_lane_end(LaneEnd(lane.get(), LaneEnd::Which::kStart));
          rg->start_bp()->set_lane_end_set_a(std::move(lane_end_set_start));
          rg->start_bp()->set_lane_end_set_b(std::make_unique<MockLaneEndSet>());
          auto lane_end_set_end = std::make_unique<MockLaneEndSet>();
          lane_end_set_end->set_lane_end(LaneEnd(lane.get(), LaneEnd::Which::kFinish));
          rg->end_bp()->set_lane_end_set_a(std::move(lane_end_set_end));
          rg->end_bp()->set_lane_end_set_b(std::make_unique<MockLaneEndSet>());
          lane->set_start_bp(rg->start_bp());
          lane->set_end_bp(rg->end_bp());
        }
        segment->set_lane(std::move(lane));
      }
      junction->set_segment(std::move(segment));
    }
    rg->set_junction(std::move(junction));
  }
  return std::move(rg);
}

std::unique_ptr<RoadGeometry> CreateOneLaneRoadGeometry() { return std::make_unique<MockOneLaneRoadGeometry>(); }

std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook() { return CreateRoadRulebook(RoadRulebookBuildFlags{}); }

std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook(const RoadRulebookBuildFlags& build_flags) {
  auto rulebook = std::make_unique<MockRoadRulebook>();
  if (build_flags.add_right_of_way) {
    rulebook->set_right_of_way(CreateRightOfWayRule(build_flags.right_of_way_build_flags));
  }
  if (build_flags.add_direction_usage) {
    rulebook->set_direction_usage(CreateDirectionUsageRule());
  }
  if (build_flags.add_speed_limit) {
    rulebook->set_speed_limit(CreateSpeedLimitRule());
  }
  return std::move(rulebook);
}

BulbGroup CreateBulbGroup(bool add_missing_bulb_group) {
  return BulbGroup(
      BulbGroup::Id{add_missing_bulb_group ? "MissingBulbGroupId" : "BulbGroupId"}, GeoPosition(), Rotation(),
      {Bulb(Bulb::Id{"BulbId"}, GeoPosition(), Rotation(), rules::BulbColor::kRed, rules::BulbType::kRound)});
}

TrafficLight CreateTrafficLight(const TrafficLightBuildFlags& build_flags) {
  return TrafficLight(
      TrafficLight::Id(build_flags.add_missing_traffic_light ? "MissingTrafficLightId" : "TrafficLightId"),
      GeoPosition(), Rotation(), {CreateBulbGroup(build_flags.add_missing_bulb_group)});
}

std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook() {
  return CreateTrafficLightBook(TrafficLightBookBuildFlags{});
}

std::unique_ptr<rules::TrafficLightBook> CreateTrafficLightBook(const TrafficLightBookBuildFlags& build_flags) {
  auto traffic_light_book = std::make_unique<MockTrafficLightBook>();
  if (build_flags.add_traffic_light) {
    traffic_light_book->set_traffic_light(CreateTrafficLight(build_flags.traffic_light_book_flags));
  }
  return std::move(traffic_light_book);
}

std::unique_ptr<rules::PhaseRingBook> CreatePhaseRingBook() { return std::make_unique<MockPhaseRingBook>(); }

std::unique_ptr<rules::RuleStateProvider> CreateRuleStateProvider() {
  return std::make_unique<MockRuleStateProvider>();
}

std::unique_ptr<rules::PhaseProvider> CreatePhaseProvider() { return std::make_unique<MockPhaseProvider>(); }

std::unique_ptr<IntersectionBook> CreateIntersectionBook() { return std::make_unique<MockIntersectionBook>(); }

}  // namespace test
}  // namespace api
}  // namespace maliput
