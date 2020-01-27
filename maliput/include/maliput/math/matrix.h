#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <initializer_list>
#include <iostream>

#include "maliput/math/vector.h"

namespace maliput {
namespace math {

/// A squared N-dimensional matrix.
/// @tparam N Indicates the dimension of the matrix.
template <std::size_t N>
class Matrix {
 public:
  /// @return A N x N identity matrix.
  static Matrix<N> Identity();

  /// Constructs a N x N matrix filled with zeros.
  Matrix();

  /// Constructs a N x N matrix from a list of double.
  ///
  /// @throw common::assertion_error When N is less than one.
  /// @throw common::assertion_error When matrix are not squared.
  Matrix(std::initializer_list<double> values);

  /// Constructs a N x N matrix from a list of Vector<N>.
  ///
  /// @throw common::assertion_error When N is less than one.
  /// @throw common::assertion_error When matrix are not squared.
  Matrix(std::initializer_list<Vector<N>> values);

  /// Constructs a N x N matrix from a array of Vector<N>.
  ///
  /// @throw common::assertion_error When N is less than one.
  /// @throw common::assertion_error When matrix are not squared.
  Matrix(std::array<Vector<N>, N> values);

  /// Copy constructor.
  Matrix(const Matrix<N>& other) = default;

  /// Move constructor.
  Matrix(Matrix<N>&& other) = default;

  /// Returns a specified row of the matrix.
  /// @param index Row number.
  /// @return A Vector<N>.
  ///
  /// @throw common::assertion_error When `index` is not less than N.
  const Vector<N> row(std::size_t index) const;

  /// Returns a specified column of the matrix.
  /// @param index Column number.
  /// @return A Vector<N>.
  ///
  /// @throw common::assertion_error When `index` is not less than N.
  const Vector<N> col(std::size_t index) const;

  /// Calculates the matrix transpose.
  /// @return A Matrix<N>.
  const Matrix<N> transpose() const;

  /// Reduce the matrix's dimension.
  /// @param row Is the number of the row to be removed.
  /// @param col Is the number of the column to be removed.
  /// @return A matrix with one less dimension.
  ///
  /// @throw common::assertion_error When `row` or `col` are out of range.
  /// @throw common::assertion_error When N is less than two.
  Matrix<N - 1> reduce(std::size_t row, std::size_t col) const;

  /// Calculates the cofactor value of a given `row` and `col`
  /// @param row Index of the matrix's rows.
  /// @param col Index of the matrix's cols.
  /// @return The cofactor value.
  ///
  /// @throw common::assertion_error When `row` or `col` are out of range.
  /// @throw common::assertion_error When N is less than one.
  double get_cofactor(std::size_t row, std::size_t col) const;

  /// Calculates the matrix determinant.
  /// @return the determinant value.
  ///
  /// @throw common::assertion_error When N is less than one.
  const double determinant() const;

  /// Calculates the cofactor matrix.
  /// @return the cofactor matrix.
  ///
  /// @throw common::assertion_error When N is less than two.
  const Matrix<N> cofactor() const;

  /// Calculates the adjoint matrix.
  /// @return the adjoint matrix.
  ///
  /// @throw common::assertion_error When N is less than two.
  const Matrix<N> adjoint() const;

  /// Calculates the matrix inverse.
  /// @return the inverse matrix.
  ///
  /// @throw common::assertion_error When N is less than two.
  /// @throw common::assertion_error When matrix is singular.
  const Matrix<N> inverse() const;

  /// Assignment operator overload.
  /// @param other Matrix<N> object.
  Matrix<N>& operator=(const Matrix<N>& other);

  /// Move assignment operator overload.
  /// @param other Matrix<N> object.
  Matrix<N>& operator=(const Matrix<N>&& other);

  /// Constant subscripting array operator overload.
  /// @param index The index of the matrix row.
  /// @return A copy of the row at `index`.
  ///
  /// @throw common::assertion_error When `index` is out of range.
  Vector<N> operator[](std::size_t index) const;

  /// Subscripting array operator overload.
  /// @param index The index of the matrix row.
  /// @return A mutable reference to the row at `index`.
  ///
  /// @throw common::assertion_error When `index` is out of range.
  Vector<N>& operator[](std::size_t index);

  /// Equality operator overload.
  bool operator==(const Matrix<N>& matrix) const;

  /// Inequality operator overload.
  bool operator!=(const Matrix<N>& matrix) const;

  /// Add operator overload.
  Matrix<N> operator+(const Matrix<N>& matrix) const;

  /// Substract operator overload.
  Matrix<N> operator-(const Matrix<N>& matrix) const;

  /// Product operator overload between Matrix<N>s.
  Matrix<N> operator*(const Matrix<N>& matrix) const;

  /// Divide operator overload between this Matrix<N> and a double.
  Matrix<N> operator/(double k) const;

  /// Product operator overload between this Matrix<N> and a Vector<N>.
  Vector<N> operator*(const Vector<N>& m_b) const;

  /// Product operator overload between a Matrix<N> and a double.
  template <std::size_t N_>
  friend Matrix<N_> operator*(const Matrix<N_>& matrix, double k);

  /// Product operator overload between a double and a Matrix<N>.
  template <std::size_t N_>
  friend Matrix<N_> operator*(double k, const Matrix<N_>& matrix);

  /// Insertion operator overload.
  template <std::size_t N_>
  friend std::ostream& operator<<(std::ostream& os, const Matrix<N_>& matrix);

 private:
  std::array<Vector<N>, N> rows_;
};

/// Convinient alias declaration.
using Matrix2 = Matrix<2>;
using Matrix3 = Matrix<3>;
using Matrix4 = Matrix<4>;

}  // namespace math
}  // namespace maliput
