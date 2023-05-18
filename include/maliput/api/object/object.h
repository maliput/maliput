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

#include <map>
#include <memory>
#include <optional>
#include <string>

#include <maliput/api/type_specific_identifier.h>
#include <maliput/common/maliput_copyable.h>
#include <maliput/math/bounding_region.h>

namespace maliput {
namespace object {
namespace api {

/// Represents an object in a given @p Coordinate system.
template <typename Coordinate>
class Object {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(Object)
  using Id = maliput::api::TypeSpecificIdentifier<class Object>;

  /// Constructs an Object.
  /// @param id Id of the object.
  /// @param properties Object's properties.
  /// @param region Object's bounding region.
  /// @tparam Coordinate Coordinate of the bounding region instance.
  Object(const Id& id, const std::map<std::string, std::string>& properties,
         std::unique_ptr<maliput::math::BoundingRegion<Coordinate>> region);

  ~Object() = default;

  /// @returns The id of the Object.
  Id id() const;

  /// @returns The bounding region of the object.
  const maliput::math::BoundingRegion<Coordinate>& bounding_region() const;

  /// @returns The position of the object in the Inertial-.
  const Coordinate& position() const;

  /// @returns A property of the object.
  std::optional<std::string> get_property(const std::string& key) const;

  /// @returns All the properties of the object.
  const std::map<std::string, std::string>& get_properties() const;

 private:
  const Id id_;
  const std::map<std::string, std::string> properties_;
  const std::unique_ptr<maliput::math::BoundingRegion<Coordinate>> region_;
};

}  // namespace api
}  // namespace object
}  // namespace maliput
