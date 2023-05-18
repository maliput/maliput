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
#include "maliput_object/api/object_book.h"

namespace maliput {
namespace object {

/// Implements api::ObjectBook for loading objects manually.
template <typename Coordinate>
class ManualObjectBook : public api::ObjectBook<Coordinate> {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ManualObjectBook)
  ManualObjectBook() = default;
  virtual ~ManualObjectBook() = default;

  /// Adds an object to the book.
  /// @param object The object to be added.
  void AddObject(std::unique_ptr<api::Object<Coordinate>> object);

  /// Removes an object from the book.
  /// @param object The object to be removed.
  void RemoveObject(const typename api::Object<Coordinate>::Id& object);

 private:
  virtual std::unordered_map<typename api::Object<Coordinate>::Id, api::Object<Coordinate>*> do_objects()
      const override;
  virtual api::Object<Coordinate>* DoFindById(const typename api::Object<Coordinate>::Id& object_id) const override;
  virtual std::vector<api::Object<Coordinate>*> DoFindByPredicate(
      std::function<bool(const api::Object<Coordinate>*)> predicate) const override;
  virtual std::vector<api::Object<Coordinate>*> DoFindOverlappingIn(
      const maliput::math::BoundingRegion<Coordinate>& region,
      const maliput::math::OverlappingType& overlapping_type) const override;

  std::unordered_map<typename api::Object<Coordinate>::Id, std::unique_ptr<api::Object<Coordinate>>> objects_;
};

}  // namespace object
}  // namespace maliput
