// BSD 3-Clause License
//
// Copyright (c) 2023, Woven Planet.
// All rights reserved.
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
#include <memory>
#include <unordered_map>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <maliput/api/lane.h>
#include <maliput/api/road_network.h>
#include <maliput/math/bounding_region.h>
#include <maliput/math/overlapping_type.h>
#include <maliput/math/vector.h>

#include "maliput_object/api/object.h"
#include "maliput_object/api/object_book.h"
#include "maliput_object/api/object_query.h"

namespace maliput {
namespace object {
namespace test_utilities {

template <typename Coordinate>
class MockObjectBook : public api::ObjectBook<Coordinate> {
 public:
  MockObjectBook() = default;
  MOCK_METHOD((std::unordered_map<typename api::Object<Coordinate>::Id, api::Object<Coordinate>*>), do_objects, (),
              (const, override));
  MOCK_METHOD((api::Object<Coordinate>*), DoFindById, (const typename api::Object<Coordinate>::Id&), (const, override));
  MOCK_METHOD((std::vector<api::Object<Coordinate>*>), DoFindByPredicate,
              (std::function<bool(const api::Object<Coordinate>*)>), (const, override));
  MOCK_METHOD((std::vector<api::Object<Coordinate>*>), DoFindOverlappingIn,
              (const maliput::math::BoundingRegion<Coordinate>&, const maliput::math::OverlappingType&),
              (const, override));
};

class MockObjectQuery : public api::ObjectQuery {
 public:
  MOCK_METHOD((std::vector<const maliput::api::Lane*>), DoFindOverlappingLanesIn,
              (const api::Object<maliput::math::Vector3>*), (const, override));
  MOCK_METHOD((std::vector<const maliput::api::Lane*>), DoFindOverlappingLanesIn,
              (const api::Object<maliput::math::Vector3>*, const maliput::math::OverlappingType&), (const, override));
  MOCK_METHOD((std::optional<const maliput::api::LaneSRoute>), DoRoute,
              (const api::Object<maliput::math::Vector3>*, const api::Object<maliput::math::Vector3>*),
              (const, override));
  MOCK_METHOD((const api::ObjectBook<maliput::math::Vector3>*), do_object_book, (), (const, override));
  MOCK_METHOD((const maliput::api::RoadNetwork*), do_road_network, (), (const, override));
};

}  // namespace test_utilities
}  // namespace object
}  // namespace maliput
