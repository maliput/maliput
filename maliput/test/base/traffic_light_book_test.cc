#include "maliput/base/traffic_light_book.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/drake_optional.h"
#include "maliput/api/lane_data.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace {

using api::rules::TrafficLight;

GTEST_TEST(TrafficLightBookTest, BasicTest) {
  const TrafficLight::Id id("my traffic light");
  const TrafficLight traffic_light(id, api::GeoPosition(10, 11, 12), api::Rotation::FromRpy(1, 2, 3), {});
  TrafficLightBook dut;

  const std::vector<TrafficLight> empty = dut.TrafficLights();
  EXPECT_EQ(empty.size(), 0);

  dut.AddTrafficLight(traffic_light);
  EXPECT_EQ(dut.GetTrafficLight(TrafficLight::Id("unknown_traffic light")), drake::nullopt);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(*dut.GetTrafficLight(id), traffic_light));

  const std::vector<TrafficLight> nonempty = dut.TrafficLights();
  EXPECT_EQ(nonempty.size(), 1);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(nonempty.at(0), traffic_light));
}

}  // namespace
}  // namespace maliput
