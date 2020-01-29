#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <iostream>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace math {

/// Base class for an N-dimensional vector.
///
/// This class follows the Curiosly Recurring Template Pattern design.
/// Instantiated templates for the following kinds of N's are provided:
/// - Vector2
/// - Vector3
/// - Vector4
///
/// @tparam N Is the dimension of the vector.
/// @tparam Derived Is the derived class type.
template <std::size_t N, typename Derived>
class VectorBase {
  // TODO(francocipollone): Move VectorBase to a VectorBase.h header.
 public:
  /// @return An N-dimensional vector filled with zeros.
  static Derived Zero();

  /// @return An N-dimensional vector filled with ones.
  static Derived Ones();

  /// Constructs a null N-dimensional vector.
  VectorBase();

  /// Constructs an N-dimensional vector from a std::array object;
  ///
  /// @throw common::assertion_error When size of `values` differs with the N parameter.
  VectorBase(std::array<double, N> values);

  /// Constructs an N-dimensional vector using initializing list;
  ///
  /// @throw common::assertion_error When size of `values` differs with the N parameter
  VectorBase(std::initializer_list<double> values);

  /// Copy constructor.
  VectorBase(const VectorBase<N, Derived>& other) = default;

  /// Move constructor.
  VectorBase(VectorBase<N, Derived>&& other) = default;

  /// @return The dot product of `*this` and `vector`.
  double dot(const VectorBase<N, Derived>& vector) const;

  /// @return A double containing the norm of the vector.
  double norm() const;

  /// Divides each vector component by `norm()`.
  void normalize();

  /// @return A Derived with *this normalized.
  Derived normalized() const;

  /// Assignment operator overload.
  /// @param other Derived object.
  Derived& operator=(const VectorBase<N, Derived>& other);

  /// Move assignment operator overload.
  /// @param other Derived object.
  Derived& operator=(const VectorBase<N, Derived>&& other);

  /// Constant subscripting array operator overload.
  /// @param index The index of the vector element.
  /// @return A copy of the value at `index`.
  ///
  /// @throw common::assertion_error When `index` is out of range.
  double operator[](std::size_t index) const;

  /// Subscripting array operator overload.
  /// @param index The index of the vector element.
  /// @return A mutable reference to the value at `index`.
  ///
  /// @throw common::assertion_error When `index` is out of range.
  double& operator[](std::size_t index);

  /// Equality operator overload.
  bool operator==(const VectorBase<N, Derived>& vector) const;

  /// Inequality operator overload.
  bool operator!=(const VectorBase<N, Derived>& vector) const;

  /// Add operator overload.
  Derived operator+(const VectorBase<N, Derived>& vector) const;

  /// Substract operator overload.
  Derived operator-(const VectorBase<N, Derived>& vector) const;

  /// Divide operator overload between a VectorBase<N, Derived> vector and a double.
  Derived operator/(double scalar) const;

  /// Product operator overload between a vector and a double.
  template <std::size_t N_, typename Derived_>
  friend Derived_ operator*(const VectorBase<N_, Derived_>& vector, double scalar);

  /// Product operator overload between a double and a vector.
  template <std::size_t N_, typename Derived_>
  friend Derived_ operator*(double scalar, const VectorBase<N_, Derived_>& vector);

  /// Insertion operator overload.
  template <std::size_t N_, typename Derived_>
  friend std::ostream& operator<<(std::ostream& os, const VectorBase<N_, Derived_>& vector);

 protected:
  std::array<double, N> values_{};
};

/// A 2-dimensional vector.
class Vector2 : public VectorBase<2, Vector2> {
 public:
  /// @return An unitary 2-dimensional vector with direction at x axis.
  static Vector2 UnitX();

  /// @return An unitary 2-dimensional vector with direction at y axis.
  static Vector2 UnitY();

  /// Constructs a null 2-dimensional vector.
  Vector2() : VectorBase<2, Vector2>() {}

  /// Constructs a 2-dimensional vector using initializing list;
  Vector2(std::initializer_list<double> values) : VectorBase<2, Vector2>(values) {}

  /// Constructs a 2-dimensional vector from a std::array object;
  Vector2(std::array<double, 2> values) : VectorBase<2, Vector2>(values) {}

  /// Constructs a 2-dimensional from double type arguments.;
  Vector2(double x, double y) : VectorBase<2, Vector2>({x, y}) {}

  /// @return The x value.
  double x() const { return values_[0]; }

  /// @return The y value.
  double y() const { return values_[1]; }

  /// @return The x value by reference.
  double& x() { return values_[0]; }

  /// @return The y value by reference.
  double& y() { return values_[1]; }
};

/// A 3-dimensional vector.
class Vector3 : public VectorBase<3, Vector3> {
 public:
  /// @return An unitary 3-dimensional vector with direction at x axis.
  static Vector3 UnitX();

  /// @return An unitary 3-dimensional vector with direction at y axis.
  static Vector3 UnitY();

  /// @return An unitary 3-dimensional vector with direction at z axis.
  static Vector3 UnitZ();

  /// Constructs a null 3-dimensional vector.
  Vector3() : VectorBase<3, Vector3>() {}

  /// Constructs a 3-dimensional vector using initializing list;
  Vector3(std::initializer_list<double> values) : VectorBase<3, Vector3>(values) {}

  /// Constructs a 3-dimensional vector from a std::array object;
  Vector3(std::array<double, 3> values) : VectorBase<3, Vector3>(values) {}

  /// Constructs a 3-dimensional from double type arguments.;
  Vector3(double x, double y, double z) : VectorBase<3, Vector3>({x, y, z}) {}

  /// @return The x value.
  double x() const { return values_[0]; }

  /// @return The y value.
  double y() const { return values_[1]; }

  /// @return The z value.
  double z() const { return values_[2]; }

  /// @return The x value by reference.
  double& x() { return values_[0]; }

  /// @return The y value by reference.
  double& y() { return values_[1]; }

  /// @return The z value by reference.
  double& z() { return values_[2]; }
};

/// A 4-dimensional vector.
class Vector4 : public VectorBase<4, Vector4> {
 public:
  /// @return An unitary 4-dimensional vector with direction at x axis.
  static Vector4 UnitX();

  /// @return An unitary 4-dimensional vector with direction at y axis.
  static Vector4 UnitY();

  /// @return An unitary 4-dimensional vector with direction at z axis.
  static Vector4 UnitZ();

  /// @return An unitary 4-dimensional vector with direction at w axis.
  static Vector4 UnitW();

  /// Constructs a null 4-dimensional vector.
  Vector4() : VectorBase<4, Vector4>() {}

  /// Constructs a 4-dimensional vector using initializing list;
  Vector4(std::initializer_list<double> values) : VectorBase<4, Vector4>(values) {}

  /// Constructs a 4-dimensional vector from a std::array object;
  Vector4(std::array<double, 4> values) : VectorBase<4, Vector4>(values) {}

  /// Constructs a 4-dimensional from double type arguments.;
  Vector4(double x, double y, double z, double w) : VectorBase<4, Vector4>({x, y, z, w}) {}

  /// @return The x value.
  double x() const { return values_[0]; }

  /// @return The y value.
  double y() const { return values_[1]; }

  /// @return The z value.
  double z() const { return values_[2]; }

  /// @return The w value.
  double w() const { return values_[3]; }

  /// @return The x value by reference.
  double& x() { return values_[0]; }

  /// @return The y value by reference.
  double& y() { return values_[1]; }

  /// @return The z value by reference.
  double& z() { return values_[2]; }

  /// @return The w value by reference.
  double& w() { return values_[3]; }
};

}  // namespace math
}  // namespace maliput
