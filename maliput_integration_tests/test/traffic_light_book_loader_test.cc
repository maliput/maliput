#include "maliput/base/traffic_light_book_loader.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "maliput/api/rules/traffic_light_book.h"
#include "maliput/api/rules/traffic_lights.h"
#include "maliput/common/filesystem.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/test_utilities/traffic_lights_compare.h"
#include "multilane/builder.h"
#include "multilane/loader.h"

namespace maliput {
namespace {

using api::GeoPosition;
using api::Rotation;
using api::rules::Bulb;
using api::rules::BulbColor;
using api::rules::BulbGroup;
using api::rules::BulbState;
using api::rules::BulbType;
using api::rules::TrafficLight;
using api::rules::TrafficLightBook;
using api::rules::UniqueBulbId;
constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

class TestLoading2x2IntersectionTrafficLightbook : public ::testing::Test {
 protected:
  TestLoading2x2IntersectionTrafficLightbook()
      : filepath_(maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_VAR) + "/2x2_intersection.yaml") {
    std::vector<std::unique_ptr<Bulb>> bulbs;
    std::vector<std::unique_ptr<BulbGroup>> bulb_groups;

    bulbs.push_back(std::make_unique<Bulb>(
        Bulb::Id("RedBulb"), GeoPosition(0, 0, 0.3937), Rotation::FromQuat(drake::Quaternion<double>(1, 0, 0, 0)),
        BulbColor::kRed, BulbType::kRound, std::nullopt, std::vector<BulbState>({BulbState::kOn, BulbState::kOff}),
        Bulb::BoundingBox()));
    bulbs.push_back(std::make_unique<Bulb>(
        Bulb::Id("YellowBulb"), GeoPosition(0, 0, 0), Rotation::FromQuat(drake::Quaternion<double>(1, 0, 0, 0)),
        BulbColor::kYellow, BulbType::kRound, std::nullopt, std::vector<BulbState>({BulbState::kOn, BulbState::kOff}),
        Bulb::BoundingBox()));
    bulbs.push_back(std::make_unique<Bulb>(
        Bulb::Id("GreenBulb"), GeoPosition(0, 0, -0.3937), Rotation::FromQuat(drake::Quaternion<double>(1, 0, 0, 0)),
        BulbColor::kGreen, BulbType::kRound, std::nullopt, std::vector<BulbState>({BulbState::kOn, BulbState::kOff}),
        Bulb::BoundingBox()));
    bulbs.push_back(std::make_unique<Bulb>(
        Bulb::Id("YellowLeftArrowBulb"), GeoPosition(0, -0.3937, -0.3937),
        Rotation::FromQuat(drake::Quaternion<double>(1, 0, 0, 0)), BulbColor::kYellow, BulbType::kArrow, 3.14,
        std::vector<BulbState>({BulbState::kOff, BulbState::kBlinking}), Bulb::BoundingBox()));
    bulb_groups.push_back(std::make_unique<BulbGroup>(BulbGroup::Id("SouthFacingBulbs"), GeoPosition(0, 0, 0),
                                                      Rotation::FromQuat(drake::Quaternion<double>(1, 0, 0, 0)),
                                                      std::move(bulbs)));
    south_facing_ = std::make_unique<const TrafficLight>(
        TrafficLight::Id("SouthFacing"), GeoPosition(1.875, 9.375, 6.0579),
        Rotation::FromQuat(drake::Quaternion<double>(0.707107, 0, 0, -0.707107)), std::move(bulb_groups));
  }

  const std::string filepath_;
  std::unique_ptr<const TrafficLight> south_facing_;
};

// Fully check the south-facing traffic light. Then spot-check the rest of them.
TEST_F(TestLoading2x2IntersectionTrafficLightbook, LoadFromFile) {
  std::unique_ptr<TrafficLightBook> book = LoadTrafficLightBookFromFile(filepath_);
  EXPECT_NE(book, nullptr);
  const TrafficLight* south_facing = book->GetTrafficLight(TrafficLight::Id("SouthFacing"));
  EXPECT_NE(south_facing, nullptr);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(south_facing, south_facing_.get()));
  for (const auto& name : {"NorthFacing", "EastFacing", "WestFacing"}) {
    EXPECT_NE(book->GetTrafficLight(TrafficLight::Id(name)), nullptr);
  }
}

}  // namespace
}  // namespace maliput
