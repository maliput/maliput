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

#include <maliput/math/vector.h>

namespace maliput {
namespace object {
namespace api {

template <typename Coordinate>
Object<Coordinate>::Object(const Id& id, const std::map<std::string, std::string>& properties,
                           std::unique_ptr<maliput::math::BoundingRegion<Coordinate>> region)
    : id_(id), properties_(properties), region_(std::move(region)) {}

template <typename Coordinate>
typename Object<Coordinate>::Id Object<Coordinate>::id() const {
  return id_;
}

template <typename Coordinate>
const maliput::math::BoundingRegion<Coordinate>& Object<Coordinate>::bounding_region() const {
  return *region_.get();
}

template <typename Coordinate>
const Coordinate& Object<Coordinate>::position() const {
  return region_->position();
};

template <typename Coordinate>
std::optional<std::string> Object<Coordinate>::get_property(const std::string& key) const {
  const auto value = properties_.find(key);
  return value != properties_.end() ? std::make_optional(value->second) : std::nullopt;
}

template <typename Coordinate>
const std::map<std::string, std::string>& Object<Coordinate>::get_properties() const {
  return properties_;
}

template class Object<maliput::math::Vector3>;

}  // namespace api
}  // namespace object
}  // namespace maliput
