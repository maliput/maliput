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
#include "maliput_object/api/object_book.h"

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
#include "maliput_object/test_utilities/mock.h"

namespace maliput {
namespace object {
namespace api {
namespace test {
namespace {

using maliput::math::Vector3;

TEST(ObjectBookTest, API) {
  test_utilities::MockObjectBook<Vector3> dut;
  const api::Object<Vector3>::Id kId{"id"};
  const maliput::math::test::MockBoundingRegion kBoundingRegion{};
  const maliput::math::OverlappingType kOverlappingType{maliput::math::OverlappingType::kContained |
                                                        maliput::math::OverlappingType::kIntersected};
  const std::unordered_map<api::Object<Vector3>::Id, api::Object<Vector3>*> kExpectedObjects{};
  std::unique_ptr<api::Object<Vector3>> kExpectedObjectById = std::make_unique<api::Object<Vector3>>(
      kId, std::map<std::string, std::string>{}, std::make_unique<maliput::math::test::MockBoundingRegion>());
  const std::vector<api::Object<Vector3>*> kExpectedObjectsByPredicate{kExpectedObjectById.get()};
  const std::vector<api::Object<Vector3>*> kExpectedObjectsByOverlapping{kExpectedObjectsByPredicate};
  const std::function<bool(const api::Object<Vector3>*)> kPredicate = [](const api::Object<Vector3>*) { return true; };
  EXPECT_CALL(dut, do_objects()).Times(1).WillOnce(::testing::Return(kExpectedObjects));
  EXPECT_CALL(dut, DoFindById(kId)).Times(1).WillOnce(::testing::Return(kExpectedObjectById.get()));
  EXPECT_CALL(dut, DoFindByPredicate(::testing::_)).Times(1).WillOnce(::testing::Return(kExpectedObjectsByPredicate));
  EXPECT_CALL(dut, DoFindOverlappingIn(::testing::_, kOverlappingType))
      .Times(1)
      .WillOnce(::testing::Return(kExpectedObjectsByOverlapping));
  EXPECT_EQ(kExpectedObjects, dut.objects());
  EXPECT_EQ(kExpectedObjectById.get(), dut.FindById(kId));
  EXPECT_EQ(kExpectedObjectsByPredicate, dut.FindByPredicate(kPredicate));
  EXPECT_EQ(kExpectedObjectsByOverlapping, dut.FindOverlappingIn(kBoundingRegion, kOverlappingType));
}

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace object
}  // namespace maliput
