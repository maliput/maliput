#include "maliput/base/phase_ring_book_loader.h"

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/base/road_rulebook_loader.h"
#include "maliput/base/simple_rulebook.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "multilane/builder.h"
#include "multilane/loader.h"
#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"

namespace maliput {
namespace {

using api::RoadGeometry;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RightOfWayRule;
using api::rules::RoadRulebook;
constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

class TestLoading2x2IntersectionPhasebook : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionPhasebook()
      : filepath_(
            maliput::common::Filesystem::get_env_path(
              MULTILANE_RESOURCE_VAR) + "/2x2_intersection.yaml"),
        road_geometry_(
            multilane::LoadFile(multilane::BuilderFactory(), filepath_)),
        rulebook_(LoadRoadRulebookFromFile(road_geometry_.get(), filepath_)),
        expected_phases_({Phase(Phase::Id("NorthSouthPhase"),
                                {{RightOfWayRule::Id("NorthStraight"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("SouthStraight"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("EastStraight"),
                                  RightOfWayRule::State::Id("Stop")},
                                 {RightOfWayRule::Id("WestStraight"),
                                  RightOfWayRule::State::Id("Stop")},
                                 {RightOfWayRule::Id("NorthRightTurn"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("SouthRightTurn"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("EastRightTurn"),
                                  RightOfWayRule::State::Id("StopThenGo")},
                                 {RightOfWayRule::Id("WestRightTurn"),
                                  RightOfWayRule::State::Id("StopThenGo")},
                                 {RightOfWayRule::Id("NorthLeftTurn"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("SouthLeftTurn"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("EastLeftTurn"),
                                  RightOfWayRule::State::Id("Stop")},
                                 {RightOfWayRule::Id("WestLeftTurn"),
                                  RightOfWayRule::State::Id("Stop")}},
                                drake::nullopt),
                          Phase(Phase::Id("EastWestPhase"),
                                {{RightOfWayRule::Id("NorthStraight"),
                                  RightOfWayRule::State::Id("Stop")},
                                 {RightOfWayRule::Id("SouthStraight"),
                                  RightOfWayRule::State::Id("Stop")},
                                 {RightOfWayRule::Id("EastStraight"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("WestStraight"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("NorthRightTurn"),
                                  RightOfWayRule::State::Id("StopThenGo")},
                                 {RightOfWayRule::Id("SouthRightTurn"),
                                  RightOfWayRule::State::Id("StopThenGo")},
                                 {RightOfWayRule::Id("EastRightTurn"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("WestRightTurn"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("NorthLeftTurn"),
                                  RightOfWayRule::State::Id("Stop")},
                                 {RightOfWayRule::Id("SouthLeftTurn"),
                                  RightOfWayRule::State::Id("Stop")},
                                 {RightOfWayRule::Id("EastLeftTurn"),
                                  RightOfWayRule::State::Id("Go")},
                                 {RightOfWayRule::Id("WestLeftTurn"),
                                  RightOfWayRule::State::Id("Go")}},
                                drake::nullopt)}){}

  const std::string filepath_;
  const std::unique_ptr<const RoadGeometry> road_geometry_;
  const std::unique_ptr<const RoadRulebook> rulebook_;
  const std::vector<Phase> expected_phases_;
};

TEST_F(TestLoading2x2IntersectionPhasebook, LoadFromFile) {
  std::unique_ptr<PhaseRingBook> phase_ring_book =
      LoadPhaseRingBookFromFile(rulebook_.get(), filepath_);
  EXPECT_NE(phase_ring_book, nullptr);
  const PhaseRing::Id ring_id("2x2Intersection");
  const drake::optional<PhaseRing> ring =
      phase_ring_book->GetPhaseRing(PhaseRing::Id(ring_id));
  EXPECT_NE(ring, drake::nullopt);
  EXPECT_EQ(ring->id(), ring_id);
  const auto& phases = ring->phases();
  EXPECT_EQ(phases.size(), 2);

  for (const auto& expected_phase : expected_phases_) {
    EXPECT_TRUE(
        MALIPUT_IS_EQUAL(expected_phase, phases.at(expected_phase.id())));
  }
}

}  // namespace
}  // namespace maliput
