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

#include <maliput/math/vector.h>

namespace maliput {
namespace object {

template <typename Coordinate>
void ManualObjectBook<Coordinate>::AddObject(std::unique_ptr<api::Object<Coordinate>> object) {
  MALIPUT_THROW_UNLESS(object != nullptr);
  objects_.emplace(object->id(), std::move(object));
}

template <typename Coordinate>
void ManualObjectBook<Coordinate>::RemoveObject(const typename api::Object<Coordinate>::Id& object) {
  MALIPUT_THROW_UNLESS(objects_.find(object) != objects_.end());
  objects_.erase(object);
}

template <typename Coordinate>
std::unordered_map<typename api::Object<Coordinate>::Id, api::Object<Coordinate>*>
ManualObjectBook<Coordinate>::do_objects() const {
  std::unordered_map<typename api::Object<Coordinate>::Id, api::Object<Coordinate>*> objects;
  objects.reserve(objects_.size());
  for (const auto& pair : objects_) {
    objects.emplace(pair.first, pair.second.get());
  }
  return objects;
}

template <typename Coordinate>
api::Object<Coordinate>* ManualObjectBook<Coordinate>::DoFindById(
    const typename api::Object<Coordinate>::Id& object_id) const {
  const auto it = objects_.find(object_id);
  return it == objects_.end() ? nullptr : it->second.get();
}

template <typename Coordinate>
std::vector<api::Object<Coordinate>*> ManualObjectBook<Coordinate>::DoFindByPredicate(
    std::function<bool(const api::Object<Coordinate>*)> predicate) const {
  std::vector<api::Object<Coordinate>*> result;
  std::for_each(objects_.begin(), objects_.end(), [&predicate, &result](const auto& pair) {
    if (predicate(pair.second.get())) {
      result.push_back(pair.second.get());
    }
  });
  return result;
}

template <typename Coordinate>
std::vector<api::Object<Coordinate>*> ManualObjectBook<Coordinate>::DoFindOverlappingIn(
    const maliput::math::BoundingRegion<Coordinate>& region,
    const maliput::math::OverlappingType& overlapping_type) const {
  std::vector<api::Object<Coordinate>*> result;
  std::for_each(objects_.begin(), objects_.end(), [&region, &overlapping_type, &result](const auto& pair) {
    if ((pair.second->bounding_region().Overlaps(region) & overlapping_type) == overlapping_type) {
      result.push_back(pair.second.get());
    }
  });
  return result;
}

template class ManualObjectBook<maliput::math::Vector3>;

}  // namespace object
}  // namespace maliput
