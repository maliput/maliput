// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2022, Toyota Research Institute. All rights reserved.
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
#include "maliput_object/base/manual_object_book.h"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <maliput/math/bounding_region.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/mock_math.h>

#include "maliput_object/api/object.h"
#include "maliput_object/api/object_book.h"

namespace maliput {
namespace object {
namespace test {
namespace {

using maliput::math::Vector3;

class ManualObjectBookTest : public ::testing::Test {
 public:
  void SetUp() override {
    EXPECT_EQ(0, static_cast<int>(dut_.objects().size()));
    dut_.AddObject(std::move(kObjectA));
    dut_.AddObject(std::move(kObjectB));
    EXPECT_EQ(2, static_cast<int>(dut_.objects().size()));
  }
  const api::Object<Vector3>::Id kIdA{"id_a"};
  const api::Object<Vector3>::Id kIdB{"id_b"};
  const std::string kPropertyA{"PropertyA"};
  const std::string kPropertyB{"PropertyB"};
  // Object A.
  std::unique_ptr<maliput::math::test::MockBoundingRegion> kRegionA =
      std::make_unique<maliput::math::test::MockBoundingRegion>();
  const maliput::math::test::MockBoundingRegion* kRegionAPtr{kRegionA.get()};
  std::unique_ptr<api::Object<Vector3>> kObjectA = std::make_unique<api::Object<Vector3>>(
      kIdA, std::map<std::string, std::string>{{kPropertyA, "DescriptionA"}}, std::move(kRegionA));
  const api::Object<Vector3>* kObjectAPtr{kObjectA.get()};

  // Object B.
  std::unique_ptr<maliput::math::test::MockBoundingRegion> kRegionB =
      std::make_unique<maliput::math::test::MockBoundingRegion>();
  const maliput::math::test::MockBoundingRegion* kRegionBPtr{kRegionB.get()};
  std::unique_ptr<api::Object<Vector3>> kObjectB = std::make_unique<api::Object<Vector3>>(
      kIdB, std::map<std::string, std::string>{{kPropertyB, "DescriptionB"}}, std::move(kRegionB));
  const api::Object<Vector3>* kObjectBPtr{kObjectB.get()};

  ManualObjectBook<Vector3> dut_;
};

TEST_F(ManualObjectBookTest, ManualObjectBook) {
  // Find by Id.
  ASSERT_EQ(kObjectAPtr->id(), dut_.FindById(kIdA)->id());
  // Find by predicate.
  const auto objects_by_predicate = dut_.FindByPredicate(
      [this](const api::Object<Vector3>* object) { return object->get_property(kPropertyB).has_value(); });
  ASSERT_EQ(1, static_cast<int>(objects_by_predicate.size()));
  EXPECT_EQ(kObjectBPtr->id(), objects_by_predicate.front()->id());

  // Find by bounding region.
  EXPECT_CALL(*kRegionAPtr, DoOverlaps(::testing::_))
      .Times(1)
      .WillOnce(::testing::Return(maliput::math::OverlappingType::kContained));
  EXPECT_CALL(*kRegionBPtr, DoOverlaps(::testing::_))
      .Times(1)
      .WillOnce(::testing::Return(maliput::math::OverlappingType::kIntersected));
  const auto objects_by_region = dut_.FindOverlappingIn(*kRegionAPtr, maliput::math::OverlappingType::kContained);
  ASSERT_EQ(1, static_cast<int>(objects_by_region.size()));
  ASSERT_EQ(kObjectAPtr->id(), objects_by_region.front()->id());

  dut_.RemoveObject(kObjectAPtr->id());
  EXPECT_EQ(1, static_cast<int>(dut_.objects().size()));
}

}  // namespace
}  // namespace test
}  // namespace object
}  // namespace maliput
