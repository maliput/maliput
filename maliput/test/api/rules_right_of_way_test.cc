/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/right_of_way_rule.h"
/* clang-format on */
// TODO(maddog@tri.global) Satisfy clang-format via rules tests directory reorg.

#include <gtest/gtest.h>

#include "maliput/api/regions.h"
#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/regions_test_utilities.h"
#include "maliput/test_utilities/rules_right_of_way_compare.h"
#include "maliput/test_utilities/rules_test_utilities.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

// Tests for RightOfWayRule::State

GTEST_TEST(RightOfWayRuleStateTest, Construction) {
  EXPECT_NO_THROW(RightOfWayRule::State(RightOfWayRule::State::Id("some_id"), RightOfWayRule::State::Type::kStop, {}));
  EXPECT_NO_THROW(RightOfWayRule::State(RightOfWayRule::State::Id("some_id"), RightOfWayRule::State::Type::kGo,
                                        api::test::YieldGroup2()));
}

GTEST_TEST(RightOfWayRuleStateTest, Accessors) {
  const RightOfWayRule::State dut(RightOfWayRule::State::Id("dut_id"), RightOfWayRule::State::Type::kStop,
                                  api::test::YieldGroup2());
  EXPECT_EQ(dut.id(), RightOfWayRule::State::Id("dut_id"));
  EXPECT_EQ(dut.type(), RightOfWayRule::State::Type::kStop);
  EXPECT_EQ(dut.yield_to(), api::test::YieldGroup2());
}

GTEST_TEST(RightOfWayRuleStateTest, Copying) {
  const RightOfWayRule::State source(RightOfWayRule::State::Id("dut_id"), RightOfWayRule::State::Type::kStopThenGo,
                                     api::test::YieldGroup2());
  const RightOfWayRule::State dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(RightOfWayRuleStateTest, Assignment) {
  const RightOfWayRule::State source(RightOfWayRule::State::Id("source_id"), RightOfWayRule::State::Type::kStopThenGo,
                                     {});
  RightOfWayRule::State dut(RightOfWayRule::State::Id("dut_id"), RightOfWayRule::State::Type::kGo,
                            api::test::YieldGroup2());
  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

// Tests for RightOfWayRule itself

GTEST_TEST(RightOfWayRuleTest, Construction) {
  EXPECT_NO_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), api::test::CreateLaneSRoute(),
                                 RightOfWayRule::ZoneType::kStopExcluded,
                                 {api::test::NoYieldState(), api::test::YieldState()}, api::test::RelatedBulbGroups()));

  EXPECT_NO_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), api::test::CreateLaneSRoute(),
                                 RightOfWayRule::ZoneType::kStopExcluded, {api::test::NoYieldState()},
                                 api::test::RelatedBulbGroups()));

  // At least one State must be provided.
  EXPECT_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), api::test::CreateLaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded, {} /* states */, api::test::RelatedBulbGroups()),
               std::exception);
  // At least one State::Id's must be unique.
  const RightOfWayRule::State kDupIdState(RightOfWayRule::State::Id("s1"), RightOfWayRule::State::Type::kGo, {});
  EXPECT_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), api::test::CreateLaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded, {api::test::NoYieldState(), kDupIdState},
                              api::test::RelatedBulbGroups()),
               std::exception);

  // BulbGroup::Ids must be unique for each TrafficLight::Id.
  const TrafficLight::Id kTrafficLightId("TrafficLightId");
  const BulbGroup::Id kBulbGroupId("BulbGroupId");
  EXPECT_THROW(RightOfWayRule(RightOfWayRule::Id("some_id"), api::test::CreateLaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded, {api::test::NoYieldState()},
                              RightOfWayRule::RelatedBulbGroups{{kTrafficLightId, {kBulbGroupId, kBulbGroupId}}}),
               maliput::common::assertion_error);
}

GTEST_TEST(RightOfWayRuleTest, Accessors) {
  const RightOfWayRule dut(RightOfWayRule::Id("dut_id"), api::test::CreateLaneSRoute(),
                           RightOfWayRule::ZoneType::kStopExcluded,
                           {api::test::NoYieldState(), api::test::YieldState()}, api::test::RelatedBulbGroups());
  EXPECT_EQ(dut.id(), RightOfWayRule::Id("dut_id"));
  EXPECT_TRUE(MALIPUT_REGIONS_IS_EQUAL(dut.zone(), api::test::CreateLaneSRoute()));
  EXPECT_EQ(dut.zone_type(), RightOfWayRule::ZoneType::kStopExcluded);
  EXPECT_EQ(dut.states().size(), 2);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.states().at(RightOfWayRule::State::Id("s1")), api::test::NoYieldState()));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.states().at(RightOfWayRule::State::Id("s2")), api::test::YieldState()));
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.related_bulb_groups(), api::test::RelatedBulbGroups()));
  EXPECT_FALSE(dut.is_static());
  EXPECT_THROW(dut.static_state(), std::exception);
}

// Specifically test the different results for a static (single state) rule.
GTEST_TEST(RightOfWayRuleTest, StaticRuleOnlyAccessors) {
  const RightOfWayRule dut(RightOfWayRule::Id("dut_id"), api::test::CreateLaneSRoute(),
                           RightOfWayRule::ZoneType::kStopExcluded, {api::test::YieldState()},
                           {} /* related_bulb_groups */);
  EXPECT_TRUE(dut.is_static());
  EXPECT_NO_THROW(dut.static_state());
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut.static_state(), api::test::YieldState()));
}

GTEST_TEST(RightOfWayRuleTest, Copying) {
  const RightOfWayRule source(RightOfWayRule::Id("source_id"), api::test::CreateLaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded,
                              {api::test::NoYieldState(), api::test::YieldState()}, {} /* related_bulb_groups */);

  const RightOfWayRule dut(source);
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

GTEST_TEST(RightOfWayRuleTest, Assignment) {
  const RightOfWayRule source(RightOfWayRule::Id("source_id"), api::test::CreateLaneSRoute(),
                              RightOfWayRule::ZoneType::kStopExcluded,
                              {api::test::NoYieldState(), api::test::YieldState()}, {} /* related_bulb_groups */);

  RightOfWayRule dut(RightOfWayRule::Id("some_id"), api::test::CreateLaneSRoute(),
                     RightOfWayRule::ZoneType::kStopAllowed, {api::test::NoYieldState()}, {} /* related_bulb_groups */);

  dut = source;
  EXPECT_TRUE(MALIPUT_IS_EQUAL(dut, source));
}

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
