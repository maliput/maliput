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
///         Dimension must be 1x1 or greater.
template <std::size_t N>
class Matrix {
 public:
  static_assert(N > 0, "0x0 Matrix are not allowed.");

  /// Tolerance value to determine the singularity of the matrix.
  static constexpr double kTolerance = 1e-15;

  /// @return A N x N identity matrix.
  static Matrix<N> Identity();

  /// Constructs a N x N matrix filled with zeros.
  Matrix() = default;

  /// Constructs a N x N matrix from a list of double.
  /// @param values Elements of the matrix. The size of `values` must match
  /// NxN. Elements will fill the matrix's rows from top to bottom.
  ///
  /// @throw common::assertion_error When N is not positive.
  /// @throw common::assertion_error When `values` size is not NxN.
  Matrix(const std::initializer_list<double>& values);

  /// Constructs a N x N matrix from a list of Vector<N>.
  /// @param rows Rows of the matrix. The size of `rows` must be N. It fills the matrix from the 0-th to N-1-th.
  ///
  /// @throw common::assertion_error When `rows` size is not N.
  Matrix(const std::initializer_list<Vector<N>>& rows);

  /// Constructs a N x N matrix from an array of Vector<N>.
  /// @param rows Rows of the matrix. The size of `rows` must match with N. It fills the matrix from top to bottom.
  Matrix(std::array<Vector<N>, N> rows);

  /// Copy constructor.
  Matrix(const Matrix<N>& other) = default;

  /// Move constructor.
  Matrix(Matrix<N>&& other) = default;

  /// Returns the `index`-th row of the matrix.
  /// @param index Row number. It must be in [0, N).
  /// @return A Vector<N>.
  ///
  /// @throw common::assertion_error When `index` is not in the range [0, N).
  const Vector<N> row(std::size_t index) const;

  /// Returns the `index`-th column of the matrix.
  /// @param index Column number. It must be in [0, N).
  /// @return A Vector<N>.
  ///
  /// @throw common::assertion_error When `index` is not in the range [0, N).
  const Vector<N> col(std::size_t index) const;

  /// @return A Matrix<N> whose elements match `this` transpose.
  Matrix<N> transpose() const;

  /// Reduce the matrix's dimension. N must be at least 2 to be reducible.
  /// @param row Is the number of the row to be removed. It must be in [0, N).
  /// @param col Is the number of the column to be removed. It must be in [0, N).
  /// @return A Matrix<N-1> which is the result of removing the `row`-th row and `col`-th column.
  ///
  /// @throw common::assertion_error When `row` or `col` are not in [0, N).
  /// @throw common::assertion_error When N is less than two.
  Matrix<N - 1> reduce(std::size_t row, std::size_t col) const;

  /// Calculates the cofactor of the element at `row` and `col`.
  /// @param row Index of the matrix's rows. It must be in [0, N).
  /// @param col Index of the matrix's cols. It must be in [0, N).
  /// @return The cofactor value.
  ///
  /// @throw common::assertion_error When `row` or `col` are not in [0, N).
  /// @throw common::assertion_error When N is one.
  double cofactor(std::size_t row, std::size_t col) const;

  /// Computes the determinant.
  /// @return The determinant value.
  double determinant() const;

  /// Determine the singularity.
  /// @return True when `this` matrix is singular.
  bool is_singular() const;

  /// @return The cofactor matrix.
  ///
  /// @throw common::assertion_error When N is one.
  Matrix<N> cofactor() const;

  /// @return The adjoint matrix.
  ///
  /// @throw common::assertion_error When N is one.
  Matrix<N> adjoint() const;

  /// @return The inverse matrix.
  ///
  /// @throw common::assertion_error When matrix is singular.
  Matrix<N> inverse() const;

  /// Assignment operator overload.
  /// @param other Matrix<N> object.
  Matrix<N>& operator=(const Matrix<N>& other);

  /// Move assignment operator overload.
  /// @param other Matrix<N> object.
  Matrix<N>& operator=(const Matrix<N>&& other);

  /// Constant subscripting array operator overload.
  /// @param index The index of the matrix row. It must be in [0, N).
  /// @return A constant reference of the row at `index`.
  ///
  /// @throw common::assertion_error When `index` is out of range.
  const Vector<N>& operator[](std::size_t index) const;

  /// Subscripting array operator overload.
  /// @param index The index of the matrix row. It must be in [0, N).
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

  /// Divide operator overload between `this` and a scalar.
  Matrix<N> operator/(double k) const;

  /// Product operator overload between a Matrix<N> and a scalar.
  template <std::size_t N_>
  friend Matrix<N_> operator*(const Matrix<N_>& matrix, double k);

  /// Product operator overload between a scalar and a Matrix<N>.
  template <std::size_t N_>
  friend Matrix<N_> operator*(double k, const Matrix<N_>& matrix);

  /// Serialization operator overload.
  template <std::size_t N_>
  friend std::ostream& operator<<(std::ostream& os, const Matrix<N_>& matrix);

 private:
  std::array<Vector<N>, N> rows_;
};

/// Product operator overload between a Matrix<N> and a VectorBase<N, Derived>.
template <std::size_t N, typename Derived>
Derived operator*(const Matrix<N>& matrix, const VectorBase<N, Derived> vector);

/// Convinient alias declaration.
using Matrix2 = Matrix<2>;
using Matrix3 = Matrix<3>;
using Matrix4 = Matrix<4>;

}  // namespace math
}  // namespace maliput
