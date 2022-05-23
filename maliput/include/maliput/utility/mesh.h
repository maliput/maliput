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
#pragma once

#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <fmt/ostream.h>

#include "maliput/api/lane.h"
#include "maliput/common/maliput_abort.h"
#include "maliput/common/maliput_hash.h"

namespace maliput {
namespace utility {
namespace mesh {

/// A container for a set of unique objects which keeps track of the original
/// insertion order.  Its primary purpose is to assign a stable unique index
/// to each element at time of insertion.
///
/// @tparam T  the inserted element type
/// @tparam Hash  a hasher suitable for std::unordered_map (e.g., std::hash)
/// @tparam KeyEqual  an equivalence relation suitable for std::unordered_map
///                   (e.g., std::equal_to)
template <class T, class Hash, class KeyEqual>
class UniqueIndexer {
 public:
  /// Creates an empty UniqueIndexer.
  UniqueIndexer() = default;

  /// Pushes @p thing onto the back of this container, and returns the unique
  /// index for @p thing.  If @p thing has already been added to the container,
  /// then this simply returns the original index for @p thing.
  int push_back(const T& thing) {
    auto mi = map_.find(thing);
    if (mi != map_.end()) {
      return mi->second;
    }
    const int index = vector_.size();
    auto it = map_.emplace(thing, index).first;
    vector_.push_back(&(it->first));
    return index;
  }

  /// Returns a vector of all elements added to this container.  The index of
  /// any element in the vector equals the index generated when the element
  /// was added to the container.
  const std::vector<const T*>& vector() const { return vector_; }

 private:
  std::unordered_map<T, int, Hash, KeyEqual> map_;
  std::vector<const T*> vector_;
};

/// An `Inertial`-frame vertex.
class GeoVertex {
 public:
  /// A hasher operation suitable for std::unordered_map.
  using Hash = maliput::common::DefaultHash;

  /// An equivalence operation suitable for std::unordered_map.
  struct Equiv {
    bool operator()(const GeoVertex& lhs, const GeoVertex& rhs) const { return (lhs.v().xyz() == rhs.v().xyz()); }
  };

  GeoVertex() {}

  explicit GeoVertex(const api::InertialPosition& v) : v_(v) {}

  const api::InertialPosition& v() const { return v_; }

  //// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const GeoVertex& item) noexcept {
    using maliput::common::hash_append;
    hash_append(hasher, item.v_.x());
    hash_append(hasher, item.v_.y());
    hash_append(hasher, item.v_.z());
  }

 private:
  api::InertialPosition v_;
};

/// An `Inertial`-frame normal vector.
class GeoNormal {
 public:
  /// A hasher operation suitable for std::unordered_map.
  using Hash = maliput::common::DefaultHash;

  /// An equivalence operation suitable for std::unordered_map.
  struct Equiv {
    bool operator()(const GeoNormal& lhs, const GeoNormal& rhs) const { return (lhs.n().xyz() == rhs.n().xyz()); }
  };

  GeoNormal() {}

  explicit GeoNormal(const api::InertialPosition& n) : n_(n) {}

  const api::InertialPosition& n() const { return n_; }

  //// Implements the @ref hash_append concept.
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const GeoNormal& item) noexcept {
    using maliput::common::hash_append;
    hash_append(hasher, item.n_.x());
    hash_append(hasher, item.n_.y());
    hash_append(hasher, item.n_.z());
  }

 private:
  api::InertialPosition n_;
};

/// An `Inertial`-frame face:  a sequence of vertices with corresponding normals.
class GeoFace {
 public:
  GeoFace() {}

  GeoFace(const std::vector<GeoVertex>& vertices, const std::vector<GeoNormal>& normals)
      : vertices_(vertices), normals_(normals) {
    MALIPUT_DEMAND(vertices.size() == normals.size());
  }

  void push_vn(const GeoVertex& vertex, const GeoNormal& normal) {
    vertices_.push_back(vertex);
    normals_.push_back(normal);
  }

  const std::vector<GeoVertex>& vertices() const { return vertices_; }

  const std::vector<GeoNormal>& normals() const { return normals_; }

 private:
  std::vector<GeoVertex> vertices_;
  std::vector<GeoNormal> normals_;
};

/// A face --- a sequence of vertices with normals --- in which the
/// vertices and normals are represented by integer indices into some
/// other container/dictionary.
class IndexFace {
 public:
  struct Vertex {
    Vertex(int vertex_index_in, int normal_index_in) : vertex_index(vertex_index_in), normal_index(normal_index_in) {}

    int vertex_index{};
    int normal_index{};
  };

  void push_vertex(int vertex_index, int normal_index) { vertices_.emplace_back(vertex_index, normal_index); }

  const std::vector<Vertex>& vertices() const { return vertices_; }

 private:
  std::vector<Vertex> vertices_;
};

/// An `Inertial`-frame mesh:  a collection of GeoFaces.
class GeoMesh {
 public:
  GeoMesh() {}

  void PushFace(const GeoFace& inertial_face) {
    IndexFace face;
    for (size_t gi = 0; gi < inertial_face.vertices().size(); ++gi) {
      int vi = vertices_.push_back(inertial_face.vertices()[gi]);
      int ni = normals_.push_back(inertial_face.normals()[gi]);
      face.push_vertex(vi, ni);
    }
    faces_.push_back(face);
  }

  void AddFacesFrom(const GeoMesh& other_mesh) {
    const std::vector<IndexFace>& other_faces = other_mesh.faces();
    const std::vector<const GeoVertex*>& other_vertices = other_mesh.vertices();
    const std::vector<const GeoNormal*>& other_normals = other_mesh.normals();
    for (const IndexFace& other_face : other_faces) {
      IndexFace face;
      const std::vector<IndexFace::Vertex>& other_face_vertices = other_face.vertices();
      for (size_t i = 0; i < other_face_vertices.size(); ++i) {
        const IndexFace::Vertex& other_face_vertex = other_face_vertices[i];
        int vi = vertices_.push_back(*other_vertices[other_face_vertex.vertex_index]);
        int ni = normals_.push_back(*other_normals[other_face_vertex.normal_index]);
        face.push_vertex(vi, ni);
      }
      faces_.push_back(face);
    }
  }

  /// Emits the mesh as Wavefront OBJ elements to @p os.  @p material is the
  /// name of an MTL-defined material to describe visual properties of the mesh.
  /// @p precision specifies the fixed-point precision (number of digits after
  /// the decimal point) for vertex and normal coordinate values.  @p origin
  /// specifies a `Inertial`-frame origin for vertex coordinates (i.e., the
  /// emitted values for a vertex at `(x,y,z)` will be `(x,y,z) - origin`).
  ///
  /// If other meshes have already been emitted to stream @p os, then
  /// @p vertex_index_offset and @p normal_index_offset must correspond to the
  /// number of previously emitted vertices and normals respectively.
  /// Conveniently, EmitObj() returns a tuple of (vertex_index_offset,
  /// normal_index_offset) which can be chained into a succeeding call to
  /// EmitObj().
  std::tuple<int, int> EmitObj(std::ostream& os, const std::string& material, int precision,
                               const api::InertialPosition& origin, int vertex_index_offset,
                               int normal_index_offset) const {
    if (faces_.empty()) {
      // Short-circuit if there is nothing to draw.
      return std::make_tuple(vertex_index_offset, normal_index_offset);
    }

    // NOLINTNEXTLINE(build/namespaces)  Usage documented by fmt library.
    using namespace fmt::literals;
    fmt::print(os, "# Vertices\n");
    for (const GeoVertex* gv : vertices_.vector()) {
      fmt::print(os, "v {x:.{p}f} {y:.{p}f} {z:.{p}f}\n", "x"_a = (gv->v().x() - origin.x()),
                 "y"_a = (gv->v().y() - origin.y()), "z"_a = (gv->v().z() - origin.z()), "p"_a = precision);
    }
    fmt::print(os, "# Normals\n");
    for (const GeoNormal* gn : normals_.vector()) {
      fmt::print(os, "vn {x:.{p}f} {y:.{p}f} {z:.{p}f}\n", "x"_a = gn->n().x(), "y"_a = gn->n().y(),
                 "z"_a = gn->n().z(), "p"_a = precision);
    }
    fmt::print(os, "\n");
    fmt::print(os, "# Faces\n");
    if (!material.empty()) {
      fmt::print(os, "usemtl {}\n", material);
    }
    for (const IndexFace& f : faces_) {
      fmt::print(os, "f");
      for (const IndexFace::Vertex& ifv : f.vertices()) {
        fmt::print(os, " {}//{}", (ifv.vertex_index + 1 + vertex_index_offset),
                   (ifv.normal_index + 1 + normal_index_offset));
      }
      fmt::print(os, "\n");
    }
    return std::make_tuple(vertex_index_offset + vertices_.vector().size(),
                           normal_index_offset + normals_.vector().size());
  }

  const std::vector<const GeoVertex*>& vertices() const { return vertices_.vector(); }

  const std::vector<const GeoNormal*>& normals() const { return normals_.vector(); }

  const std::vector<IndexFace>& faces() const { return faces_; }

  int num_vertices() const { return vertices_.vector().size(); }
  int num_normals() const { return normals_.vector().size(); }
  const GeoVertex& get_vertex(int vi) const { return *vertices_.vector()[vi]; }
  const GeoNormal& get_normal(int ni) const { return *normals_.vector()[ni]; }

 private:
  UniqueIndexer<GeoVertex, GeoVertex::Hash, GeoVertex::Equiv> vertices_;
  UniqueIndexer<GeoNormal, GeoNormal::Hash, GeoNormal::Equiv> normals_;
  std::vector<IndexFace> faces_;
};

/// A `Lane`-frame face: a sequence of vertices expressed in the (s,r,h)
/// coordinates of an api::Lane (which is not referenced here).  Each
/// vertex has a normal vector also expressed in the `Lane`-frame.
class SrhFace {
 public:
  SrhFace(const std::initializer_list<api::LanePosition> vertices, const api::LanePosition& normal)
      : vertices_(vertices), normal_(normal) {}

  /// Given a @p lane, calculates the corresponding GeoFace.
  GeoFace ToGeoFace(const api::Lane* lane) const {
    GeoFace inertial_face;
    for (const api::LanePosition& srh : vertices_) {
      api::InertialPosition xyz(lane->ToInertialPosition(srh));
      api::InertialPosition n = api::InertialPosition::FromXyz(lane->GetOrientation(srh).quat() * normal_.srh());
      inertial_face.push_vn(GeoVertex(xyz), GeoNormal(n));
    }
    return inertial_face;
  }

 private:
  std::vector<api::LanePosition> vertices_;
  api::LanePosition normal_;
};

}  // namespace mesh
}  // namespace utility
}  // namespace maliput
