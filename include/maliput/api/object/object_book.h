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
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>

#include <maliput/common/maliput_copyable.h>
#include <maliput/math/bounding_region.h>
#include <maliput/math/overlapping_type.h>

#include "maliput_object/api/object.h"

namespace maliput {
namespace object {
namespace api {

/// Book for Objects in a given @p Coordinate system.
/// TODO(#14): ObjectBook should be capable of holding all Objects regardless of the Coordinate that
///            determines their spatial characteristics. When finding by regions it should be able of filtering by
///            coordinate type.
template <typename Coordinate>
class ObjectBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ObjectBook)
  virtual ~ObjectBook() = default;

  /// Gets all the Objects in the book.
  /// @returns A unordered map, from which key and value are Object::Id and pointer to Object respectively.
  std::unordered_map<typename Object<Coordinate>::Id, Object<Coordinate>*> objects() const { return do_objects(); }

  /// Finds Object by Id.
  /// @param object_id An Object::Id.
  /// @returns A valid Object's pointer if found, nullptr otherwise.
  Object<Coordinate>* FindById(const typename Object<Coordinate>::Id& object_id) const { return DoFindById(object_id); }

  /// Finds the Objects that make @p predicate true.
  /// @param predicate Unary predicate for evaluating an object.
  std::vector<Object<Coordinate>*> FindByPredicate(std::function<bool(const Object<Coordinate>*)> predicate) const {
    return DoFindByPredicate(predicate);
  }

  /// Finds the Objects that intersect with a @p region according to certain @p overlapping_type.
  /// @param region BoundaryRegion used for finding intersected Objects.
  /// @param overlapping_type Indicates type of overlapping. See #OverlappingType.
  /// @returns The Objects intersecting @p region with the given @p overlapping_type .
  std::vector<Object<Coordinate>*> FindOverlappingIn(const maliput::math::BoundingRegion<Coordinate>& region,
                                                     const maliput::math::OverlappingType& overlapping_type) const {
    return DoFindOverlappingIn(region, overlapping_type);
  }

 protected:
  ObjectBook() = default;

 private:
  virtual std::unordered_map<typename Object<Coordinate>::Id, Object<Coordinate>*> do_objects() const = 0;
  virtual Object<Coordinate>* DoFindById(const typename Object<Coordinate>::Id& object_id) const = 0;
  virtual std::vector<Object<Coordinate>*> DoFindByPredicate(
      std::function<bool(const Object<Coordinate>*)> predicate) const = 0;
  virtual std::vector<Object<Coordinate>*> DoFindOverlappingIn(
      const maliput::math::BoundingRegion<Coordinate>& region,
      const maliput::math::OverlappingType& overlapping_type) const = 0;
};

}  // namespace api
}  // namespace object
}  // namespace maliput
