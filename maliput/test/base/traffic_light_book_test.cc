#include "maliput/base/traffic_light_book.h"

#include <gtest/gtest.h>

#include "maliput/api/lane_data.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/test_utilities/traffic_lights_compare.h"

namespace maliput {
namespace {

using api::rules::BulbGroup;
using api::rules::TrafficLight;

GTEST_TEST(TrafficLightBookTest, BasicTest) {
  const TrafficLight::Id id("my traffic light");
  std::vector<std::unique_ptr<BulbGroup>> empty_bulb_group{};
  auto traffic_light = std::make_unique<const TrafficLight>(
      id, api::GeoPosition(10, 11, 12), api::Rotation::FromRpy(1, 2, 3), std::move(empty_bulb_group));
  const TrafficLight* traffic_light_ptr = traffic_light.get();

  TrafficLightBook dut;

  const std::vector<const TrafficLight*> empty = dut.TrafficLights();
  EXPECT_EQ(empty.size(), 0);

  dut.AddTrafficLight(std::move(traffic_light));
  EXPECT_EQ(dut.GetTrafficLight(TrafficLight::Id("unknown_traffic light")), nullptr);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.GetTrafficLight(id), traffic_light_ptr));

  const std::vector<const TrafficLight*> nonempty = dut.TrafficLights();
  EXPECT_EQ(nonempty.size(), 1);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(nonempty.at(0), traffic_light_ptr));
}

}  // namespace
}  // namespace maliput
