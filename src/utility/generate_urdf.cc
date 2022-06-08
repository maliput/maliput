// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2019-2022, Toyota Research Institute. All rights reserved.
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
#include "maliput/utility/generate_urdf.h"

#include <fstream>

#include <fmt/ostream.h>

#include "maliput/utility/generate_obj.h"

namespace maliput {
namespace utility {

void GenerateUrdfFile(const api::RoadGeometry* road_geometry, const std::string& dirpath, const std::string& fileroot,
                      const ObjFeatures& features) {
  GenerateObjFile(road_geometry, dirpath, fileroot, features);

  const std::string obj_filename = fileroot + ".obj";
  const std::string urdf_filename = fileroot + ".urdf";

  std::ofstream os(dirpath + "/" + urdf_filename, std::ios::binary);
  fmt::print(os,
             R"X(<?xml version="1.0" ?>
<robot name="{0}">
  <link name="world"/>

  <joint name="world_to_road_joint" type="continuous">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="world"/>
    <child link="surface"/>
  </joint>

  <link name="surface">
    <visual name="v1">
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="{1}" scale="1.0 1.0 1.0"/>
      </geometry>
    </visual>
  </link>
</robot>
)X",
             road_geometry->id().string(), obj_filename);
}

}  // namespace utility
}  // namespace maliput
