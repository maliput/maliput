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

#include <memory>
#include <string>

#include <maliput/math/vector.h>

#include "maliput_object/api/object_book.h"

/// @file loader.h
/// @page maliput_object_yaml_spec Maliput Object YAML specification
/// @author Agustin Alba Chicar
/// @author Franco Cipollone
/// @date May 12, 2022
/// @tableofcontents
///
/// @section maliput_object_yaml YAML format specification for objects
///
/// The maliput::object API provides two functions to load a concrete
/// maliput::object::api::ObjectBook implementation from a YAML description:
/// - @ref maliput::object::loader::LoadFile() for YAML files.
/// - @ref maliput::object::loader::Load() for string serialized YAML descriptions.
///
/// The only supported type of coordinate is maliput::math::Vector3 , meaning
/// that concrete @ref maliput::math::BoundingRegion "BoundingRegions"
/// are limited to @ref maliput::math::BoundingBox "BoundingBox".
///
/// @subsection maliput_object_yaml_format YAML format
///
/// See the following code snippet that shows an example of a YAML ObjectBook:
///
/// @code{.yml}
/// maliput_objects:
///   my_awesome_object:
///     bounding_region:
///       position: [1.2, 3.4, 4.5]
///       rotation: [0.1745329, 0.3490659, 0.5235988]
///       type: box
///       box_size: [1.1, 2.2, 3.3]
///     properties:
///       - user_defined_prop_1: my_value_1
///         ...
///         ...
///       - user_defined_prop_n: my_value_n
///   ...
/// @endcode
///
/// @subsubsection maliput_object_yaml_entities Entities
///
/// <h4>`maliput_objects`</h4>
///
/// `maliput_object` is a dictionary of objects. Every first level key
/// identifies a new @ref maliput::object::api::Object :
///
/// @code{.yml}
///
/// maliput_object:
///   object_1:
///      ...
///   object_2:
///      ...
///   object_n:
///      ...
///
/// @endcode
///
/// <h4>`bounding_region`</h4>
///
/// A mapping describing a bounding region. For boxes:
///
/// @code{.yml}
///
/// bounding_region:
///   position: [X, Y, Z]
///   rotation: [R, P, Y]
///   type: box
///   box_size: [L, D, H]
///
/// @endcode
///
/// Where:
///
/// - `position`: is the Inertial Frame position described as a 3D Vector: @f$[X, Y, Z]@f$.
/// - `rotation`: is the Inertial Frame rotation described as a 3D Vector of Euler angles
///   @f$[R, P, Y]@f$ (roll, pitch, yaw).
/// - `type`: must be `box`.
/// - `box_size`: the size of the bounding box expressed as if the box is at the origin of
///   the Inertial Frame and aligned with it. Each coordinate means  @f$[Length, Depth, Height]@f$.
///
/// <h4>`properties`</h4>
///
/// A listing of user defined properties. The listing is flat, meaning that it
/// is a collection of key-value pairs with no depth. Keys and values will be
/// read as strings.
///
/// @code{.yml}
///
/// properties:
///   - user_defined_prop_1: my_value_1
///   ...
///
///   - user_defined_prop_n: my_value_n
///
/// @endcode
///
/// Where:
///
/// - `user_defined_prop_1` and `user_defined_prop_n` are examples of keys.
/// - `my_value_1` and `my_value_n` are examples of values.
namespace maliput {
namespace object {
namespace loader {

/// Loads the @p input string as a `maliput_object` YAML document.
/// See @ref loader.h documentation for further details.
///
/// @param input A YAML document as a string that must contain a node as described in @ref loader.h
/// @throws maliput::common::assertion_error When @p input fails to be parsed.
/// @return A maliput::object::api::ObjectBook<maliput::math::Vector3> representing the @p input.
std::unique_ptr<maliput::object::api::ObjectBook<maliput::math::Vector3>> Load(const std::string& input);

/// Loads the @p filename file as a `maliput_object` YAML document.
/// See @ref loader.h documentation for further details.
///
/// @param filename The path to the YAML document.
/// @throws maliput::common::assertion_error When @p input fails to be parsed.
/// @return A maliput::object::api::ObjectBook<maliput::math::Vector3> representing the contents of @p filename.
std::unique_ptr<maliput::object::api::ObjectBook<maliput::math::Vector3>> LoadFile(const std::string& filename);

}  // namespace loader
}  // namespace object
}  // namespace maliput
