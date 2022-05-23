// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
/* clang-format off to disable clang-format-includes */
#include "maliput/api/rules/traffic_lights.h"
/* clang-format on */
// TODO(liang.fok) Satisfy clang-format via rules tests directory reorg.

#include <algorithm>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "maliput/common/assertion_error.h"
#include "maliput/test_utilities/mock.h"
#include "maliput/test_utilities/rules_test_utilities.h"
#include "maliput/test_utilities/traffic_lights_compare.h"

namespace maliput {
namespace api {
namespace rules {
namespace {

GTEST_TEST(BulbColorTest, InstantiateAndAssign) {
  BulbColor dut{};
  EXPECT_EQ(dut, BulbColor::kRed);
  for (BulbColor color : {BulbColor::kGreen, BulbColor::kYellow}) {
    EXPECT_NE(dut, color);
    dut = color;
    EXPECT_EQ(dut, color);
  }
}

GTEST_TEST(BulbColorTest, MapperTest) {
  const auto dut = BulbColorMapper();
  const std::vector<BulbColor> expected_colors{BulbColor::kRed, BulbColor::kYellow, BulbColor::kGreen};
  EXPECT_EQ(dut.size(), expected_colors.size());
  for (BulbColor color : expected_colors) {
    EXPECT_EQ(static_cast<int>(dut.count(color)), 1);
  }
}

GTEST_TEST(BulbTypeTest, InstantiateAndAssign) {
  BulbType dut{};
  EXPECT_EQ(dut, BulbType::kRound);
  for (BulbType type : {BulbType::kArrow}) {
    EXPECT_NE(dut, type);
    dut = type;
    EXPECT_EQ(dut, type);
  }
}

GTEST_TEST(BulbTypeTest, MapperTest) {
  const auto dut = BulbTypeMapper();
  const std::vector<BulbType> expected_types{BulbType::kRound, BulbType::kArrow};
  EXPECT_EQ(dut.size(), expected_types.size());
  for (BulbType type : expected_types) {
    EXPECT_EQ(static_cast<int>(dut.count(type)), 1);
  }
}

GTEST_TEST(BulbStateTest, InstantiateAndAssign) {
  BulbState dut{};
  EXPECT_EQ(dut, BulbState::kOff);
  for (BulbState state : {BulbState::kOn, BulbState::kBlinking}) {
    EXPECT_NE(dut, state);
    dut = state;
    EXPECT_EQ(dut, state);
  }
}

GTEST_TEST(BulbStateTest, MapperTest) {
  const auto dut = BulbStateMapper();
  const std::vector<BulbState> expected_states{BulbState::kOff, BulbState::kOn, BulbState::kBlinking};
  EXPECT_EQ(dut.size(), expected_states.size());
  for (BulbState state : expected_states) {
    EXPECT_EQ(static_cast<int>(dut.count(state)), 1);
  }
}

GTEST_TEST(BulbConstructorTest, ArrowWithoutOrientation) {
  EXPECT_THROW(Bulb(Bulb::Id("other_dut_id"), InertialPosition(7, 8, 9), Rotation::FromRpy(10, 11, 12),
                    BulbColor::kGreen, BulbType::kArrow),
               std::exception);
}

GTEST_TEST(BulbConstructorTest, NonArrowWithOrientation) {
  EXPECT_THROW(Bulb(Bulb::Id("other_dut_id"), InertialPosition(7, 8, 9), Rotation::FromRpy(10, 11, 12),
                    BulbColor::kGreen, BulbType::kRound, 0 /* arrow_orientation_rad */),
               std::exception);
}

GTEST_TEST(BulbConstructorTest, EmptyAndNullOptStateVector) {
  std::vector<std::unique_ptr<Bulb>> test_cases;
  test_cases.push_back(std::make_unique<Bulb>(Bulb::Id("empty_state_vector"), InertialPosition(0, 0, 0),
                                              Rotation::FromRpy(0, 0, 0), BulbColor::kGreen, BulbType::kRound,
                                              std::nullopt /* arrow_orientation_rad */, std::vector<BulbState>{}));
  test_cases.push_back(std::make_unique<Bulb>(Bulb::Id("std::nullopt_state_vector"), InertialPosition(0, 0, 0),
                                              Rotation::FromRpy(0, 0, 0), BulbColor::kGreen, BulbType::kRound,
                                              std::nullopt /* arrow_orientation_rad */, std::nullopt /* states */));

  for (const auto& test_case : test_cases) {
    EXPECT_EQ(static_cast<int>(test_case->states().size()), 2);
    EXPECT_EQ(test_case->GetDefaultState(), BulbState::kOff);
    EXPECT_TRUE(test_case->IsValidState(BulbState::kOff));
    EXPECT_TRUE(test_case->IsValidState(BulbState::kOn));
    EXPECT_FALSE(test_case->IsValidState(BulbState::kBlinking));
  }
}

class BulbTest : public ::testing::Test {
 public:
  BulbTest()
      : bulb_(Bulb::Id("dut_id"), InertialPosition(1, 2, 3), Rotation::FromRpy(4, 5, 6), BulbColor::kRed,
              BulbType::kRound) {}
  const Bulb bulb_;
};

TEST_F(BulbTest, Accessors) {
  EXPECT_EQ(bulb_.id(), Bulb::Id("dut_id"));
  EXPECT_THROW(bulb_.unique_id(), common::assertion_error);
  EXPECT_EQ(bulb_.position_bulb_group(), InertialPosition(1, 2, 3));
  EXPECT_EQ(bulb_.orientation_bulb_group().matrix(), Rotation::FromRpy(4, 5, 6).matrix());
  EXPECT_EQ(bulb_.color(), BulbColor::kRed);
  EXPECT_EQ(bulb_.type(), BulbType::kRound);
  EXPECT_EQ(static_cast<int>(bulb_.states().size()), 2);
  EXPECT_EQ(bulb_.states().at(0), BulbState::kOff);
  EXPECT_EQ(bulb_.states().at(1), BulbState::kOn);
  EXPECT_EQ(bulb_.GetDefaultState(), BulbState::kOff);
  EXPECT_TRUE(bulb_.IsValidState(BulbState::kOff));
  EXPECT_TRUE(bulb_.IsValidState(BulbState::kOn));
  EXPECT_FALSE(bulb_.IsValidState(BulbState::kBlinking));
  MALIPUT_IS_EQUAL(bulb_.bounding_box(), Bulb::BoundingBox());
}

GTEST_TEST(DefaultBulbStateTest, CorrectDefaultAndIsValidStateQueries) {
  struct TestCase {
    std::vector<BulbState> states;
    BulbState default_state;
  };
  const std::vector<TestCase> test_cases = {{{BulbState::kBlinking, BulbState::kOn}, BulbState::kBlinking},
                                            {{BulbState::kOn, BulbState::kBlinking}, BulbState::kBlinking},
                                            {{BulbState::kBlinking, BulbState::kOff}, BulbState::kOff},
                                            {{BulbState::kOff, BulbState::kBlinking}, BulbState::kOff},
                                            {{BulbState::kOff, BulbState::kOn}, BulbState::kOff},
                                            {{BulbState::kOn, BulbState::kOff}, BulbState::kOff},
                                            {{BulbState::kOn, BulbState::kOff, BulbState::kBlinking}, BulbState::kOff},
                                            {{BulbState::kOff, BulbState::kOn, BulbState::kBlinking}, BulbState::kOff},
                                            {{BulbState::kOff, BulbState::kBlinking, BulbState::kOn}, BulbState::kOff},
                                            {{BulbState::kBlinking, BulbState::kOn, BulbState::kOff}, BulbState::kOff},
                                            {{BulbState::kBlinking, BulbState::kOff, BulbState::kOn}, BulbState::kOff},
                                            {{BulbState::kBlinking}, BulbState::kBlinking},
                                            {{BulbState::kOn}, BulbState::kOn},
                                            {{BulbState::kOff}, BulbState::kOff}};
  const Bulb::Id b_id("id");
  for (const auto& test_case : test_cases) {
    const Bulb dut(b_id, InertialPosition(0, 0, 0), Rotation::FromRpy(0, 0, 0), BulbColor::kGreen, BulbType::kRound,
                   std::nullopt /* arrow_orientation_rad */, test_case.states);
    EXPECT_EQ(dut.GetDefaultState(), test_case.default_state);
    for (const auto& state : test_case.states) {
      EXPECT_TRUE(dut.IsValidState(state));
    }
  }
}

class BulbGroupConstructorTest : public ::testing::Test {
 protected:
  const BulbGroup::Id kDutId{"dut_id"};
  const InertialPosition kDutPosition{1., 2., 3.};
  const Rotation kDutRotation{Rotation::FromRpy(0, 0, 0)};
  const InertialPosition kZeroPosition{0., 0., 0.};
  const Rotation kZeroRotation{Rotation::FromRpy(0, 0, 0)};
  const Bulb::Id kBulbId{"bulb_id"};
};

TEST_F(BulbGroupConstructorTest, InvalidGroupSize) {
  EXPECT_THROW(BulbGroup(kDutId, kDutPosition, kDutRotation, {}), std::exception);
}

TEST_F(BulbGroupConstructorTest, DuplicatedBulbIds) {
  std::vector<std::unique_ptr<Bulb>> bulbs;
  bulbs.push_back(std::make_unique<Bulb>(kBulbId, kZeroPosition, kZeroRotation, BulbColor::kRed, BulbType::kRound));
  bulbs.push_back(
      std::make_unique<Bulb>(kBulbId, InertialPosition(0, 0, 0.3), kZeroRotation, BulbColor::kGreen, BulbType::kRound));
  EXPECT_THROW(BulbGroup(kDutId, kDutPosition, kDutRotation, std::move(bulbs)), common::assertion_error);
}

TEST_F(BulbGroupConstructorTest, NullBulb) {
  std::vector<std::unique_ptr<Bulb>> bulbs;
  bulbs.push_back(std::make_unique<Bulb>(kBulbId, kZeroPosition, kZeroRotation, BulbColor::kRed, BulbType::kRound));
  bulbs.push_back({});

  EXPECT_THROW(BulbGroup(kDutId, kDutPosition, kDutRotation, std::move(bulbs)), common::assertion_error);
}

class BulbGroupTest : public ::testing::Test {
 public:
  const BulbGroup::Id kBulbGroupId{"test_bulb_group"};
  const InertialPosition kBulbGroupPosition{1., 2., 3.};
  const Rotation kBulbGroupRotation{Rotation::FromRpy(4., 5., 6.)};

  BulbGroupTest() {
    std::vector<std::unique_ptr<Bulb>> bulbs;
    bulbs.push_back(std::make_unique<Bulb>(Bulb::Id("red_bulb"), InertialPosition(0, 0, -0.25),
                                           Rotation::FromRpy(0, 0, 0), BulbColor::kRed, BulbType::kRound));
    red_bulb_ptr_ = bulbs.back().get();

    bulbs.push_back(std::make_unique<Bulb>(Bulb::Id("yellow_bulb"), InertialPosition(0, 0, 0),
                                           Rotation::FromRpy(0, 0, 0), BulbColor::kYellow, BulbType::kRound));
    yellow_bulb_ptr_ = bulbs.back().get();

    bulbs.push_back(std::make_unique<Bulb>(Bulb::Id("green_bulb"), InertialPosition(0, 0, 0.25),
                                           Rotation::FromRpy(0, 0, 0), BulbColor::kGreen, BulbType::kRound));
    green_bulb_ptr_ = bulbs.back().get();

    bulb_group_ = std::make_unique<BulbGroup>(kBulbGroupId, kBulbGroupPosition, kBulbGroupRotation, std::move(bulbs));
  }

  const Bulb* red_bulb_ptr_{};
  const Bulb* yellow_bulb_ptr_{};
  const Bulb* green_bulb_ptr_{};
  std::unique_ptr<BulbGroup> bulb_group_;
};

TEST_F(BulbGroupTest, Accessors) {
  EXPECT_EQ(bulb_group_->id(), kBulbGroupId);
  EXPECT_EQ(bulb_group_->position_traffic_light(), kBulbGroupPosition);
  EXPECT_EQ(bulb_group_->orientation_traffic_light().matrix(), kBulbGroupRotation.matrix());
  EXPECT_EQ(static_cast<int>(bulb_group_->bulbs().size()), 3);
  EXPECT_EQ(bulb_group_->GetBulb(Bulb::Id("unknown_bulb")), nullptr);
  EXPECT_EQ(bulb_group_->GetBulb(Bulb::Id("red_bulb")), red_bulb_ptr_);
  EXPECT_THROW(bulb_group_->unique_id(), common::assertion_error);
}

class TrafficLightConstructorTest : public ::testing::Test {
 public:
  const TrafficLight::Id kDutId{"traffic_light_id"};
  const BulbGroup::Id kBulbGroupId{"bulb_group_id"};
  const Bulb::Id kGreenBulbId{"green_bulb"};
  const Bulb::Id kRedBulbId{"red_bulb"};
  const InertialPosition kZeroPosition{0., 0., 0.};
  const Rotation kZeroRotation{Rotation::FromRpy(0., 0., 0.)};
};

TEST_F(TrafficLightConstructorTest, DuplicatedBulbGroupIds) {
  std::vector<std::unique_ptr<BulbGroup>> bulb_group;
  bulb_group.push_back(api::test::CreateBulbGroup(false /* add_missing_bulb_group */));
  bulb_group.push_back(api::test::CreateBulbGroup(false /* add_missing_bulb_group */));
  EXPECT_THROW(TrafficLight(kDutId, kZeroPosition, kZeroRotation, std::move(bulb_group)), common::assertion_error);
}

TEST_F(TrafficLightConstructorTest, NullBulbGroup) {
  std::vector<std::unique_ptr<Bulb>> bulbs;
  std::vector<std::unique_ptr<BulbGroup>> bulb_group;
  bulbs.push_back(std::make_unique<Bulb>(kRedBulbId, kZeroPosition, kZeroRotation, BulbColor::kRed, BulbType::kRound));
  bulb_group.push_back(std::make_unique<BulbGroup>(kBulbGroupId, kZeroPosition, kZeroRotation, std::move(bulbs)));
  bulb_group.push_back({});
  EXPECT_THROW(TrafficLight(kDutId, kZeroPosition, kZeroRotation, std::move(bulb_group)), common::assertion_error);
}

class TrafficLightTest : public ::testing::Test {
 public:
  const InertialPosition kZeroPosition{0., 0., 0.};
  const Rotation kZeroRotation{Rotation::FromRpy(0., 0., 0.)};
  const InertialPosition kTrafficLightPosition{0, 0, 5};
  const Rotation kTrafficLightRotation{kZeroRotation};
  const TrafficLight::Id kId{"four_way_stop"};

  TrafficLightTest() {
    std::vector<std::unique_ptr<BulbGroup>> bulb_group;

    {
      std::vector<std::unique_ptr<Bulb>> bulbs;
      bulbs.push_back(std::make_unique<Bulb>(Bulb::Id("north_bulb"), kZeroPosition, kZeroRotation, BulbColor::kRed,
                                             BulbType::kRound));
      bulb_group.push_back(std::make_unique<BulbGroup>(BulbGroup::Id("north_group"), InertialPosition(0, 0.1, 0),
                                                       Rotation::FromRpy(0, 0, M_PI_2), std::move(bulbs)));
      north_bulb_group_ = bulb_group.back().get();
    }
    {
      std::vector<std::unique_ptr<Bulb>> bulbs;
      bulbs.push_back(std::make_unique<Bulb>(Bulb::Id("south_bulb"), kZeroPosition, kZeroRotation, BulbColor::kRed,
                                             BulbType::kRound));
      bulb_group.push_back(std::make_unique<BulbGroup>(BulbGroup::Id("south_group"), InertialPosition(0, -0.1, 0),
                                                       Rotation::FromRpy(0, 0, -M_PI_2), std::move(bulbs)));
      south_bulb_group_ = bulb_group.back().get();
    }
    {
      std::vector<std::unique_ptr<Bulb>> bulbs;
      bulbs.push_back(std::make_unique<Bulb>(Bulb::Id("east_bulb"), kZeroPosition, kZeroRotation, BulbColor::kRed,
                                             BulbType::kRound));
      bulb_group.push_back(std::make_unique<BulbGroup>(BulbGroup::Id("east_group"), InertialPosition(0.1, 0., 0),
                                                       kZeroRotation, std::move(bulbs)));
      east_bulb_group_ = bulb_group.back().get();
    }
    {
      std::vector<std::unique_ptr<Bulb>> bulbs;
      bulbs.push_back(std::make_unique<Bulb>(Bulb::Id("west_bulb"), kZeroPosition, kZeroRotation, BulbColor::kRed,
                                             BulbType::kRound));
      bulb_group.push_back(std::make_unique<BulbGroup>(BulbGroup::Id("west_group"), InertialPosition(-0.1, 0., 0),
                                                       Rotation::FromRpy(0, 0, M_PI), std::move(bulbs)));
      west_bulb_group_ = bulb_group.back().get();
    }
    traffic_light_ =
        std::make_unique<TrafficLight>(kId, kTrafficLightPosition, kTrafficLightRotation, std::move(bulb_group));
  }

  const BulbGroup* north_bulb_group_{};
  const BulbGroup* south_bulb_group_{};
  const BulbGroup* east_bulb_group_{};
  const BulbGroup* west_bulb_group_{};
  std::unique_ptr<const TrafficLight> traffic_light_;
};

TEST_F(TrafficLightTest, Accessors) {
  EXPECT_EQ(traffic_light_->id(), kId);
  EXPECT_EQ(traffic_light_->position_road_network(), kTrafficLightPosition);
  EXPECT_EQ(traffic_light_->orientation_road_network().matrix(), kTrafficLightRotation.matrix());
  EXPECT_EQ(static_cast<int>(traffic_light_->bulb_groups().size()), 4);
  EXPECT_EQ(traffic_light_->GetBulbGroup(BulbGroup::Id("unknown_bulb_group")), nullptr);
  EXPECT_EQ(traffic_light_->GetBulbGroup(BulbGroup::Id("north_group")), north_bulb_group_);
}

TEST_F(TrafficLightTest, UniqueIds) {
  EXPECT_EQ(traffic_light_->GetBulbGroup(BulbGroup::Id("north_group"))->unique_id(),
            UniqueBulbGroupId(kId, BulbGroup::Id("north_group")));
  EXPECT_EQ(traffic_light_->GetBulbGroup(BulbGroup::Id("north_group"))->GetBulb(Bulb::Id("north_bulb"))->unique_id(),
            UniqueBulbId(kId, BulbGroup::Id("north_group"), Bulb::Id("north_bulb")));
}

GTEST_TEST(UniqueBulbIdTest, DefaultConstructor) {
  const UniqueBulbId dut;
  EXPECT_EQ(dut.traffic_light_id(), TrafficLight::Id("default"));
  EXPECT_EQ(dut.bulb_group_id(), BulbGroup::Id("default"));
  EXPECT_EQ(dut.bulb_id(), Bulb::Id("default"));
}

GTEST_TEST(UniqueBulbIdTest, Usage) {
  const std::string traffic_light_name{"MyTrafficLight"};
  const std::string bulb_group_name{"MyBulbGroup"};
  const std::string bulb_name{"MyBulb"};

  const TrafficLight::Id traffic_light_id(traffic_light_name);
  const BulbGroup::Id bulb_group_id(bulb_group_name);
  const Bulb::Id bulb_id(bulb_name);

  const UniqueBulbId dut(traffic_light_id, bulb_group_id, bulb_id);

  // A mismatch of just one internal ID results in the UniqueBulbId no longer
  // matching.
  EXPECT_NE(dut, (UniqueBulbId{TrafficLight::Id("foo"), bulb_group_id, bulb_id}));
  EXPECT_NE(dut, (UniqueBulbId{traffic_light_id, BulbGroup::Id("foo"), bulb_id}));
  EXPECT_NE(dut, (UniqueBulbId{traffic_light_id, bulb_group_id, Bulb::Id("foo")}));

  const std::string dut_string = dut.string();
  for (const auto& name : {traffic_light_name, bulb_group_name, bulb_name}) {
    EXPECT_NE(dut_string.find(name), std::string::npos);
  }

  const UniqueBulbId copied_dut = dut;
  EXPECT_EQ(copied_dut, dut);

  UniqueBulbId assigned_dut{TrafficLight::Id("foo"), BulbGroup::Id("bar"), Bulb::Id("baz")};
  EXPECT_NE(assigned_dut, dut);
  assigned_dut = dut;
  EXPECT_EQ(assigned_dut, dut);

  std::unordered_map<UniqueBulbId, BulbState> unordered_map;
  const BulbState bulb_state = BulbState::kOn;
  unordered_map.emplace(std::make_pair(dut, bulb_state));
  EXPECT_NE(unordered_map.find(dut), unordered_map.end());
  EXPECT_EQ(unordered_map.at(dut), bulb_state);
  const UniqueBulbId other_dut{TrafficLight::Id("foo"), BulbGroup::Id("bar"), Bulb::Id("baz")};
  EXPECT_EQ(unordered_map.find(other_dut), unordered_map.end());

  std::map<UniqueBulbId, BulbState> ordered_map;
  ordered_map.emplace(std::make_pair(dut, bulb_state));
  EXPECT_NE(ordered_map.find(dut), ordered_map.end());
  EXPECT_EQ(ordered_map.at(dut), bulb_state);
  EXPECT_EQ(ordered_map.find(other_dut), ordered_map.end());

  // Tests the std::less<UniqueBulbId>() operator.
  auto make_less = [](const std::string& s) -> std::string {
    std::string c = s;
    // Letter "L" is considered less than letter  "M" because its ASCII code is
    // less (0x4C vs. 0x4D).
    std::replace(c.begin(), c.end(), 'M', 'L');
    return c;
  };
  auto make_more = [](const std::string& s) -> std::string {
    std::string c = s;
    // Letter "N" is greater than "M" because its ASCII code is higher (0x4E vs.
    // 0x4D).
    std::replace(c.begin(), c.end(), 'M', 'N');
    return c;
  };
  const TrafficLight::Id less_traffic_light_id(make_less(traffic_light_name));
  const TrafficLight::Id more_traffic_light_id(make_more(traffic_light_name));
  const BulbGroup::Id less_bulb_group_id(make_less(bulb_group_name));
  const BulbGroup::Id more_bulb_group_id(make_more(bulb_group_name));
  const Bulb::Id less_bulb_id(make_less(bulb_name));
  const Bulb::Id more_bulb_id(make_more(bulb_name));

  const std::less<UniqueBulbId> less;

  const std::vector<UniqueBulbId> less_set = {{less_traffic_light_id, bulb_group_id, bulb_id},
                                              {less_traffic_light_id, less_bulb_group_id, more_bulb_id},
                                              {less_traffic_light_id, more_bulb_group_id, more_bulb_id},
                                              {less_traffic_light_id, more_bulb_group_id, less_bulb_id},
                                              {traffic_light_id, less_bulb_group_id, bulb_id},
                                              {traffic_light_id, less_bulb_group_id, more_bulb_id},
                                              {traffic_light_id, bulb_group_id, less_bulb_id}};

  for (const auto& test_case : less_set) {
    EXPECT_TRUE(less(test_case, dut));
  }

  const std::vector<UniqueBulbId> not_less_set = {dut,
                                                  {more_traffic_light_id, less_bulb_group_id, less_bulb_id},
                                                  {more_traffic_light_id, more_bulb_group_id, less_bulb_id},
                                                  {more_traffic_light_id, less_bulb_group_id, more_bulb_id},
                                                  {more_traffic_light_id, more_bulb_group_id, more_bulb_id},
                                                  {traffic_light_id, more_bulb_group_id, more_bulb_id},
                                                  {traffic_light_id, more_bulb_group_id, less_bulb_id},
                                                  {traffic_light_id, bulb_group_id, more_bulb_id}};

  for (const auto& test_case : not_less_set) {
    EXPECT_FALSE(less(test_case, dut));
  }
}

GTEST_TEST(UniqueBulbIdTest, Delimiter) { EXPECT_EQ(UniqueBulbId::delimiter(), "-"); }

GTEST_TEST(UniqueBulbGroupIdTest, DefaultConstructor) {
  const UniqueBulbGroupId dut;
  EXPECT_EQ(dut.traffic_light_id(), TrafficLight::Id("default"));
  EXPECT_EQ(dut.bulb_group_id(), BulbGroup::Id("default"));
}

GTEST_TEST(UniqueBulbGroupIdTest, Usage) {
  const std::string traffic_light_name{"MyTrafficLight"};
  const std::string bulb_group_name{"MyBulbGroup"};

  const TrafficLight::Id traffic_light_id(traffic_light_name);
  const BulbGroup::Id bulb_group_id(bulb_group_name);

  const UniqueBulbGroupId dut(traffic_light_id, bulb_group_id);

  // A mismatch of just one internal ID results in the UniqueBulbGroupId no
  // longer matching.
  EXPECT_NE(dut, UniqueBulbGroupId(TrafficLight::Id("foo"), bulb_group_id));
  EXPECT_NE(dut, UniqueBulbGroupId(traffic_light_id, BulbGroup::Id("foo")));

  const std::string dut_string = dut.string();
  for (const auto& name : {traffic_light_name, bulb_group_name}) {
    EXPECT_NE(dut_string.find(name), std::string::npos);
  }

  const UniqueBulbGroupId copied_dut = dut;
  EXPECT_EQ(copied_dut, dut);

  UniqueBulbGroupId assigned_dut{TrafficLight::Id("foo"), BulbGroup::Id("bar")};
  EXPECT_NE(assigned_dut, dut);
  assigned_dut = dut;
  EXPECT_EQ(assigned_dut, dut);

  const int kOne{1};  // Used as mock value by the following collections.
  std::unordered_map<UniqueBulbGroupId, int> unordered_map;
  unordered_map.emplace(std::make_pair(dut, kOne));
  EXPECT_NE(unordered_map.find(dut), unordered_map.end());
  EXPECT_EQ(unordered_map.at(dut), kOne);
  const UniqueBulbGroupId other_dut(TrafficLight::Id("foo"), BulbGroup::Id("bar"));
  EXPECT_EQ(unordered_map.find(other_dut), unordered_map.end());

  std::map<UniqueBulbGroupId, int> ordered_map;
  ordered_map.emplace(std::make_pair(dut, kOne));
  EXPECT_NE(ordered_map.find(dut), ordered_map.end());
  EXPECT_EQ(ordered_map.at(dut), kOne);
  EXPECT_EQ(ordered_map.find(other_dut), ordered_map.end());

  // Tests the std::less<UniqueBulbGroupId>() operator.
  auto make_less = [](const std::string& s) -> std::string {
    std::string c = s;
    // Letter "L" is considered less than letter  "M" because its ASCII code is
    // less (0x4C vs. 0x4D).
    std::replace(c.begin(), c.end(), 'M', 'L');
    return c;
  };
  auto make_more = [](const std::string& s) -> std::string {
    std::string c = s;
    // Letter "N" is greater than "M" because its ASCII code is higher (0x4E vs.
    // 0x4D).
    std::replace(c.begin(), c.end(), 'M', 'N');
    return c;
  };
  const TrafficLight::Id less_traffic_light_id(make_less(traffic_light_name));
  const TrafficLight::Id more_traffic_light_id(make_more(traffic_light_name));
  const BulbGroup::Id less_bulb_group_id(make_less(bulb_group_name));
  const BulbGroup::Id more_bulb_group_id(make_more(bulb_group_name));

  const std::less<UniqueBulbGroupId> less;

  const std::vector<UniqueBulbGroupId> less_set{{less_traffic_light_id, bulb_group_id},
                                                {less_traffic_light_id, less_bulb_group_id},
                                                {less_traffic_light_id, more_bulb_group_id},
                                                {traffic_light_id, less_bulb_group_id}};

  for (const auto& test_case : less_set) {
    EXPECT_TRUE(less(test_case, dut));
  }

  const std::vector<UniqueBulbGroupId> not_less_set{dut,
                                                    {more_traffic_light_id, less_bulb_group_id},
                                                    {more_traffic_light_id, more_bulb_group_id},
                                                    {traffic_light_id, more_bulb_group_id}};

  for (const auto& test_case : not_less_set) {
    EXPECT_FALSE(less(test_case, dut));
  }
}

GTEST_TEST(UniqueBulbGroupIdTest, Delimiter) { EXPECT_EQ(UniqueBulbGroupId::delimiter(), "-"); }

}  // namespace
}  // namespace rules
}  // namespace api
}  // namespace maliput
