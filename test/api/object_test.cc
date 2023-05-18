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
#include "maliput_object/api/object.h"

#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <maliput/math/bounding_region.h>
#include <maliput/math/vector.h>
#include <maliput/test_utilities/mock_math.h>

namespace maliput {
namespace object {
namespace api {
namespace test {
namespace {

using maliput::math::Vector3;

class ObjectTest : public ::testing::Test {
 public:
  const Object<Vector3>::Id kId{"ObjectTest"};
  const Vector3 kExpectedPosition{1., 2., 3.};
  const maliput::math::OverlappingType kExpectedOverlapping{maliput::math::OverlappingType::kContained};
  const std::map<std::string, std::string> kExpectedProperties{{"Key1", "Value1"}, {"Key2", "Value2"}};
  std::unique_ptr<maliput::math::test::MockBoundingRegion> region_{
      std::make_unique<maliput::math::test::MockBoundingRegion>()};
  const maliput::math::test::MockBoundingRegion* region_ptr_{region_.get()};
};

TEST_F(ObjectTest, Constructor) { EXPECT_NO_THROW(Object<Vector3>(kId, {}, std::move(region_))); }

TEST_F(ObjectTest, API) {
  const api::Object<Vector3> dut{kId, kExpectedProperties, std::move(region_)};
  ASSERT_EQ(kId, dut.id());

  EXPECT_CALL(*region_ptr_, DoOverlaps(::testing::_)).Times(1).WillOnce(::testing::Return(kExpectedOverlapping));
  EXPECT_CALL(*region_ptr_, do_position()).Times(1).WillOnce(::testing::ReturnRef(kExpectedPosition));
  ASSERT_EQ(kExpectedOverlapping, dut.bounding_region().Overlaps(maliput::math::test::MockBoundingRegion{}));
  ASSERT_EQ(kExpectedPosition, dut.position());

  ASSERT_EQ(kExpectedProperties, dut.get_properties());
  const std::string kValidPropertyKey{kExpectedProperties.begin()->first};
  ASSERT_EQ(kExpectedProperties.at(kValidPropertyKey), dut.get_property(kValidPropertyKey));
  const std::string kInvalidPropertyKey{"invalid_key"};
  ASSERT_EQ(std::nullopt, dut.get_property(kInvalidPropertyKey));
}

}  // namespace
}  // namespace test
}  // namespace api
}  // namespace object
}  // namespace maliput
