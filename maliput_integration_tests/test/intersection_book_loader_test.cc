#include "maliput/base/intersection_book_loader.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "maliput/api/intersection_book.h"
#include "maliput/api/road_geometry.h"
#include "maliput/api/rules/phase.h"
#include "maliput/api/rules/phase_ring.h"
#include "maliput/api/rules/phase_ring_book.h"
#include "maliput/api/rules/road_rulebook.h"
#include "maliput/api/rules/traffic_light_book.h"
#include "maliput/base/manual_phase_provider.h"
#include "maliput/base/phase_ring_book_loader.h"
#include "maliput/base/road_rulebook_loader.h"
#include "maliput/base/traffic_light_book_loader.h"
#include "maliput/common/filesystem.h"
#include "multilane/builder.h"
#include "multilane/loader.h"

namespace maliput {
namespace {

using api::Intersection;
using api::IntersectionBook;
using api::RoadGeometry;
using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::PhaseRingBook;
using api::rules::RoadRulebook;
using api::rules::TrafficLightBook;

constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

class TestLoading2x2IntersectionIntersectionBook : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionIntersectionBook()
      : filepath_(maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_VAR) + "/2x2_intersection.yaml"),
        road_geometry_(multilane::LoadFile(multilane::BuilderFactory(), filepath_)),
        rulebook_(LoadRoadRulebookFromFile(road_geometry_.get(), filepath_)),
        traffic_light_book_(LoadTrafficLightBookFromFile(filepath_)),
        ring_book_(LoadPhaseRingBookFromFile(rulebook_.get(), traffic_light_book_.get(), filepath_)) {}

  const std::string filepath_;
  const std::unique_ptr<const RoadGeometry> road_geometry_;
  const std::unique_ptr<const RoadRulebook> rulebook_;
  const std::unique_ptr<const TrafficLightBook> traffic_light_book_;
  const std::unique_ptr<const PhaseRingBook> ring_book_;
};

TEST_F(TestLoading2x2IntersectionIntersectionBook, LoadFromFile) {
  const PhaseRing::Id ring_id("2x2Intersection");
  const drake::optional<PhaseRing> ring = ring_book_->GetPhaseRing(PhaseRing::Id(ring_id));
  EXPECT_TRUE(ring.has_value());

  ManualPhaseProvider phase_provider;
  std::unique_ptr<api::IntersectionBook> book =
      LoadIntersectionBookFromFile(filepath_, *rulebook_, *ring_book_, &phase_provider);
  EXPECT_NE(book, nullptr);
  EXPECT_EQ(int(book->GetIntersections().size()), 1);
  EXPECT_EQ(book->GetIntersection(Intersection::Id("unknown")), nullptr);
  Intersection* intersection = book->GetIntersection(Intersection::Id("2x2Intersection"));
  EXPECT_NE(intersection, nullptr);
  EXPECT_TRUE(intersection->Phase().has_value());
  EXPECT_EQ(intersection->Phase()->state, Phase::Id("NorthSouthPhase"));
  EXPECT_TRUE(intersection->Phase()->next.has_value());
  EXPECT_GT(intersection->region().size(), 0);
  EXPECT_EQ(intersection->ring_id(), PhaseRing::Id("2x2Intersection"));
}

}  // namespace
}  // namespace maliput
