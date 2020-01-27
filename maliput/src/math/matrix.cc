#include "maliput/math/matrix.h"

#include <cmath>

#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace math {

template <std::size_t N>
Matrix<N> Matrix<N>::Identity() {
  Matrix<N> matrix{};
  for (std::size_t i = 0; i < N; i++) {
    matrix[i][i] = 1.;
  }
  return matrix;
}

template <std::size_t N>
Matrix<N>::Matrix() : rows_() {
  rows_.fill(Vector<N>{});
}

template <std::size_t N>
Matrix<N>::Matrix(std::initializer_list<double> values) {
  MALIPUT_THROW_UNLESS(N >= 1);
  MALIPUT_THROW_UNLESS(values.size() == N * N);
  std::size_t count_values{};
  std::size_t count_rows{};
  std::array<double, N> vector;
  for (const auto& value : values) {
    vector[count_values] = value;
    count_values++;
    if (count_values >= N) {
      rows_[count_rows] = vector;
      count_rows++;
      count_values = 0;
    }
  }
}

template <std::size_t N>
Matrix<N>::Matrix(std::initializer_list<Vector<N>> values) {
  MALIPUT_THROW_UNLESS(N >= 1);
  MALIPUT_THROW_UNLESS(values.size() == N);
  std::size_t count_rows{};
  for (const auto& value : values) {
    rows_[count_rows] = value;
    count_rows++;
  }
}

template <std::size_t N>
Matrix<N>::Matrix(std::array<Vector<N>, N> values) : rows_(values) {
  MALIPUT_THROW_UNLESS(N >= 1);
  MALIPUT_THROW_UNLESS(values.size() == N);
}

template <std::size_t N>
const Vector<N> Matrix<N>::row(std::size_t index) const {
  MALIPUT_THROW_UNLESS(index < N);
  return rows_[index];
}

template <std::size_t N>
const Vector<N> Matrix<N>::col(std::size_t index) const {
  MALIPUT_THROW_UNLESS(index < N);
  Vector<N> vector{};
  std::size_t count_rows{};
  for (const auto& row : rows_) {
    vector[count_rows] = row[index];
    count_rows++;
  }
  return vector;
}

template <std::size_t N>
const Matrix<N> Matrix<N>::transpose() const {
  std::array<Vector<N>, N> transposed{};
  for (std::size_t i = 0; i < N; i++) {
    transposed[i] = col(i);
  }
  return transposed;
}

template <std::size_t N>
Matrix<N - 1> Matrix<N>::reduce(std::size_t row_index, std::size_t col_index) const {
  MALIPUT_THROW_UNLESS(row_index < N);
  MALIPUT_THROW_UNLESS(col_index < N);
  MALIPUT_THROW_UNLESS(N > 1);
  std::array<Vector<N - 1>, N - 1> reduced;
  std::size_t count_rows{};
  bool remove{true};
  for (const auto& row : rows_) {
    if (remove && count_rows == row_index) {
      remove = false;
      continue;
    }
    reduced[count_rows] = row.reduce(col_index);
    count_rows++;
  }
  return reduced;
}

template <std::size_t N>
double Matrix<N>::get_cofactor(std::size_t row_index, std::size_t col_index) const {
  MALIPUT_THROW_UNLESS(row_index < N);
  MALIPUT_THROW_UNLESS(col_index < N);
  MALIPUT_THROW_UNLESS(N > 1);
  return std::pow(-1, row_index + col_index) * (this->reduce(row_index, col_index).determinant());
}

template <std::size_t N>
const double Matrix<N>::determinant() const {
  MALIPUT_THROW_UNLESS(N >= 1);
  double result{0};
  switch (rows_.size()) {
    case 2:
      result = rows_[0][0] * rows_[1][1] - rows_[0][1] * rows_[1][0];
      break;
    case 1:
      return rows_[0][0];
      break;
    default:
      for (std::size_t j = 0; j < N; j++) {
        result += rows_[0][j] * get_cofactor(0, j);
      }
      break;
  }
  return result;
}

template <std::size_t N>
const Matrix<N> Matrix<N>::cofactor() const {
  Matrix<N> cofactor_matrix{};
  for (std::size_t i = 0; i < N; i++) {
    for (std::size_t j = 0; j < N; j++) {
      cofactor_matrix[i][j] = get_cofactor(i, j);
    }
  }
  return cofactor_matrix;
}

template <std::size_t N>
const Matrix<N> Matrix<N>::adjoint() const {
  return cofactor().transpose();
}

template <std::size_t N>
const Matrix<N> Matrix<N>::inverse() const {
  const double kDet = determinant();
  if (kDet == 0) MALIPUT_THROW_MESSAGE("Imposible to calculate the inverse of singular matrix.");
  return {(1 / kDet) * adjoint()};
}

template <std::size_t N>
Matrix<N>& Matrix<N>::operator=(const Matrix<N>& other) {
  if (this != &other) rows_ = other.rows_;
  return *this;
}

template <std::size_t N>
Matrix<N>& Matrix<N>::operator=(const Matrix<N>&& other) {
  if (this != &other) rows_ = other.rows_;
  return *this;
}

template <std::size_t N>
Vector<N>& Matrix<N>::operator[](std::size_t index) {
  MALIPUT_THROW_UNLESS(index < N);
  return rows_[index];
}

template <std::size_t N>
Vector<N> Matrix<N>::operator[](std::size_t index) const {
  MALIPUT_THROW_UNLESS(index < N);
  return rows_[index];
}

template <std::size_t N>
bool Matrix<N>::operator==(const Matrix<N>& matrix) const {
  return std::equal(rows_.cbegin(), rows_.cend(), matrix.rows_.cbegin());
}

template <std::size_t N>
bool Matrix<N>::operator!=(const Matrix<N>& matrix) const {
  return !(*this == matrix);
}

template <std::size_t N>
Matrix<N> Matrix<N>::operator+(const Matrix<N>& matrix) const {
  std::array<Vector<N>, N> sum{};
  std::transform(rows_.cbegin(), rows_.cend(), matrix.rows_.cbegin(), sum.begin(), std::plus<Vector<N>>());
  return sum;
}

template <std::size_t N>
Matrix<N> Matrix<N>::operator-(const Matrix<N>& matrix) const {
  std::array<Vector<N>, N> sum{};
  std::transform(rows_.cbegin(), rows_.cend(), matrix.rows_.cbegin(), sum.begin(), std::minus<Vector<N>>());
  return sum;
}

template <std::size_t N>
Matrix<N> Matrix<N>::operator*(const Matrix<N>& matrix) const {
  Matrix<N> result{};
  for (std::size_t i = 0; i < N; i++) {
    for (std::size_t j = 0; j < N; j++) {
      result[i][j] = row(i).dot(matrix.col(j));
    }
  }
  return result;
}

template <std::size_t N>
Matrix<N> Matrix<N>::operator/(double k) const {
  std::array<Vector<N>, N> res{};
  std::transform(rows_.cbegin(), rows_.cend(), res.begin(), [k](const Vector<N>& vector) { return vector / k; });
  return res;
}

template <std::size_t N>
Vector<N> Matrix<N>::operator*(const Vector<N>& vector) const {
  std::array<double, N> res{};
  std::transform(rows_.cbegin(), rows_.cend(), res.begin(), [vector](const Vector<N>& row) { return row.dot(vector); });
  return res;
}

template <std::size_t N_>
Matrix<N_> operator*(const Matrix<N_>& matrix, double k) {
  std::array<Vector<N_>, N_> res{};
  std::transform(matrix.rows_.cbegin(), matrix.rows_.cend(), res.begin(),
                 [k](const Vector<N_>& vector) { return vector * k; });
  return res;
}

template <std::size_t N_>
Matrix<N_> operator*(double k, const Matrix<N_>& matrix) {
  return matrix * k;
}

template <std::size_t N_>
std::ostream& operator<<(std::ostream& os, const Matrix<N_>& matrix) {
  os << "{";
  for (const auto& row : matrix.rows_) {
    os << row;
    if (*(std::prev(matrix.rows_.cend())) == row) break;
    os << ",\n ";
  }
  os << "}";
  return os;
}

// Specialization for Matrix<N>::get_cofactor method with N=1;
template <>
double Matrix<1>::get_cofactor(std::size_t row, std::size_t col) const {
  return 0.;
}

// Matrix<N> Explicit instanciations.
template Matrix<2> operator*(const Matrix<2>&, double);
template Matrix<3> operator*(const Matrix<3>&, double);
template Matrix<4> operator*(const Matrix<4>&, double);
template Matrix<2> operator*(double, const Matrix<2>&);
template Matrix<3> operator*(double, const Matrix<3>&);
template Matrix<4> operator*(double, const Matrix<4>&);
template std::ostream& operator<<(std::ostream&, const Matrix<2>&);
template std::ostream& operator<<(std::ostream&, const Matrix<3>&);
template std::ostream& operator<<(std::ostream&, const Matrix<4>&);
template class Matrix<1>;
template class Matrix<2>;
template class Matrix<3>;
template class Matrix<4>;

}  // namespace math
}  // namespace maliput
