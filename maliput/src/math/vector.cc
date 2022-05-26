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
#include "maliput/math/vector.h"

#include <complex>
#include <functional>
#include <limits>
#include <numeric>

namespace maliput {
namespace math {

template <std::size_t N, typename Derived>
Derived VectorBase<N, Derived>::Ones() {
  std::array<double, N> ret;
  ret.fill(1);
  return Derived{ret};
}

template <std::size_t N, typename Derived>
Derived VectorBase<N, Derived>::Zero() {
  std::array<double, N> ret;
  ret.fill(0);
  return Derived{ret};
}

template <std::size_t N, typename Derived>
Derived VectorBase<N, Derived>::FromStr(const std::string& vector_str) {
  static constexpr char kLeftBrace = '{';
  static constexpr char kRightBrace = '}';
  static constexpr char kComma = ',';

  // @{ Checks vector_str format.
  MALIPUT_THROW_UNLESS(std::count(vector_str.begin(), vector_str.end(), kLeftBrace) == 1);
  MALIPUT_THROW_UNLESS(std::count(vector_str.begin(), vector_str.end(), kRightBrace) == 1);
  MALIPUT_THROW_UNLESS(std::count(vector_str.begin(), vector_str.end(), kComma) == N - 1);
  // @}

  std::array<double, N> ret;
  if (ret.size() == 1) {
    // 1-D vector.
    const std::string value_str = vector_str.substr(1, vector_str.find_first_of(kRightBrace) - 1);
    ret[0] = std::stod(value_str);
  } else {
    std::size_t previous_comma_pos{0};
    for (size_t i = 0; i < ret.size(); ++i) {
      // Extracts each number.
      std::string value_str{};
      const std::size_t comma_pos = vector_str.find_first_of(kComma, previous_comma_pos + 1);
      if (i == ret.size() - 1) {
        // Last value.
        value_str = vector_str.substr(previous_comma_pos + 1, vector_str.find_first_of(kRightBrace) - 1);
      } else {
        value_str = vector_str.substr(previous_comma_pos + 1, comma_pos - 1);
      }

      ret[i] = std::stod(value_str);
      previous_comma_pos = comma_pos;
    }
  }
  return Derived{ret};
}

template <std::size_t N, typename Derived>
VectorBase<N, Derived>::VectorBase() : values_(std::array<double, N>{}) {
  values_.fill(0);
}

template <std::size_t N, typename Derived>
VectorBase<N, Derived>::VectorBase(std::array<double, N> values) : values_(values) {
  MALIPUT_THROW_UNLESS(values.size() == N);
}

template <std::size_t N, typename Derived>
VectorBase<N, Derived>::VectorBase(std::initializer_list<double> values) {
  MALIPUT_THROW_UNLESS(values.size() == N);
  std::copy_n(values.begin(), N, values_.begin());
}

template <std::size_t N, typename Derived>
double VectorBase<N, Derived>::dot(const VectorBase<N, Derived>& vector) const {
  return std::inner_product(vector.values_.cbegin(), vector.values_.cend(), values_.cbegin(), 0.);
}

template <std::size_t N, typename Derived>
double VectorBase<N, Derived>::norm() const {
  return std::sqrt(std::inner_product(values_.cbegin(), values_.cend(), values_.cbegin(), 0.));
}

template <std::size_t N, typename Derived>
void VectorBase<N, Derived>::normalize() {
  const double l = norm();
  std::transform(values_.begin(), values_.end(), values_.begin(), [l](double value) { return value / l; });
}

template <std::size_t N, typename Derived>
Derived VectorBase<N, Derived>::normalized() const {
  return Derived(*this / norm());
}

template <std::size_t N, typename Derived>
std::string VectorBase<N, Derived>::to_str() const {
  std::stringstream ss;
  ss.precision(std::numeric_limits<double>::digits10);
  ss << "{";
  for (size_t i = 0; i < values_.size(); ++i) {
    if (i != 0) {
      ss << ", ";
    }
    ss << values_[i];
  }
  ss << "}";
  return ss.str();
}

template <std::size_t N, typename Derived>
Derived& VectorBase<N, Derived>::operator=(const VectorBase<N, Derived>& other) {
  if (this != &other) values_ = other.values_;
  return static_cast<Derived&>(*this);
}

template <std::size_t N, typename Derived>
Derived& VectorBase<N, Derived>::operator=(const VectorBase<N, Derived>&& other) {
  if (this != &other) values_ = other.values_;
  return static_cast<Derived&>(*this);
}

template <std::size_t N, typename Derived>
double VectorBase<N, Derived>::operator[](std::size_t index) const {
  MALIPUT_THROW_UNLESS(index >= 0);
  MALIPUT_THROW_UNLESS(index < N);
  return values_[index];
}

template <std::size_t N, typename Derived>
double& VectorBase<N, Derived>::operator[](std::size_t index) {
  MALIPUT_THROW_UNLESS(index >= 0);
  MALIPUT_THROW_UNLESS(index < N);
  return values_[index];
}

template <std::size_t N, typename Derived>
bool VectorBase<N, Derived>::operator==(const VectorBase<N, Derived>& vector) const {
  return std::equal(values_.cbegin(), values_.cend(), vector.values_.cbegin());
}

template <std::size_t N, typename Derived>
bool VectorBase<N, Derived>::operator!=(const VectorBase<N, Derived>& vector) const {
  return !(*this == vector);
}

template <std::size_t N, typename Derived>
Derived VectorBase<N, Derived>::operator+(const VectorBase<N, Derived>& vector) const {
  std::array<double, N> res{};
  std::transform(values_.cbegin(), values_.cend(), vector.values_.cbegin(), res.begin(), std::plus<double>());
  return Derived(res);
}

template <std::size_t N, typename Derived>
Derived& VectorBase<N, Derived>::operator+=(const VectorBase<N, Derived>& vector) {
  std::array<double, N> res{};
  std::transform(values_.cbegin(), values_.cend(), vector.values_.cbegin(), res.begin(), std::plus<double>());
  values_ = res;
  return static_cast<Derived&>(*this);
}

template <std::size_t N, typename Derived>
Derived VectorBase<N, Derived>::operator-(const VectorBase<N, Derived>& vector) const {
  std::array<double, N> res{};
  std::transform(values_.cbegin(), values_.cend(), vector.values_.cbegin(), res.begin(), std::minus<double>());
  return Derived(res);
}

template <std::size_t N, typename Derived>
Derived VectorBase<N, Derived>::operator/(double scalar) const {
  std::array<double, N> res{};
  std::transform(values_.cbegin(), values_.cend(), res.begin(), [scalar](double value) { return value / scalar; });
  return {res};
}

template <std::size_t N_, typename Derived_>
Derived_ operator*(const VectorBase<N_, Derived_>& vector, double scalar) {
  std::array<double, N_> res{};
  std::transform(vector.values_.cbegin(), vector.values_.cend(), res.begin(),
                 [scalar](double value) { return value * scalar; });
  return Derived_(res);
}

template <std::size_t N_, typename Derived_>
Derived_ operator*(double scalar, const VectorBase<N_, Derived_>& vector) {
  return vector * scalar;
}

template <std::size_t N_, typename Derived_>
std::ostream& operator<<(std::ostream& os, const VectorBase<N_, Derived_>& vector) {
  os << vector.to_str();
  return os;
}

template <size_t N>
Vector<N - 1> Vector<N>::reduce(size_t index) const {
  MALIPUT_THROW_UNLESS(N >= 2);
  MALIPUT_THROW_UNLESS(index < N);
  std::array<double, N - 1> reduced{};
  size_t count_elements{};
  bool remove{true};
  for (const auto& value : this->values_) {
    if (remove && count_elements == index) {
      remove = false;
      continue;
    }
    reduced[count_elements] = value;
    count_elements++;
  }
  return reduced;
}

Vector2 Vector2::UnitX() { return {1., 0.}; }

Vector2 Vector2::UnitY() { return {0., 1.}; }

Vector3 Vector3::UnitX() { return {1., 0., 0.}; }

Vector3 Vector3::UnitY() { return {0., 1., 0.}; }

Vector3 Vector3::UnitZ() { return {0., 0., 1.}; }

Vector3 Vector3::cross(const Vector3& v) const {
  return {y() * v.z() - z() * v.y(), z() * v.x() - x() * v.z(), x() * v.y() - y() * v.x()};
}

Vector4 Vector4::UnitX() { return {1., 0., 0., 0.}; }

Vector4 Vector4::UnitY() { return {0., 1., 0., 0.}; }

Vector4 Vector4::UnitZ() { return {0., 0., 1., 0.}; }

Vector4 Vector4::UnitW() { return {0., 0., 0., 1.}; }

// Vector<N> Explicit instanciations.
template Vector<1> operator*(const VectorBase<1, Vector<1>>&, double);
template Vector<1> operator*(double, const VectorBase<1, Vector<1>>&);
template Vector<2> operator*(const VectorBase<2, Vector<2>>&, double);
template Vector<2> operator*(double, const VectorBase<2, Vector<2>>&);
template Vector<3> operator*(const VectorBase<3, Vector<3>>&, double);
template Vector<3> operator*(double, const VectorBase<3, Vector<3>>&);
template Vector<4> operator*(const VectorBase<4, Vector<4>>&, double);
template Vector<4> operator*(double, const VectorBase<4, Vector<4>>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<1, Vector<1>>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<2, Vector<2>>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<3, Vector<3>>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<4, Vector<4>>&);
template class Vector<1>;
template class Vector<2>;
template class Vector<3>;
template class Vector<4>;
template class VectorBase<1, Vector<1>>;
template class VectorBase<2, Vector<2>>;
template class VectorBase<3, Vector<3>>;
template class VectorBase<4, Vector<4>>;

// Vector2 Explicit instanciations.
template Vector2 operator*(const VectorBase<2, Vector2>&, double);
template Vector2 operator*(double, const VectorBase<2, Vector2>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<2, Vector2>&);
template class VectorBase<2, Vector2>;

// Vector3 Explicit instanciations.
template Vector3 operator*(const VectorBase<3, Vector3>&, double);
template Vector3 operator*(double, const VectorBase<3, Vector3>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<3, Vector3>&);
template class VectorBase<3, Vector3>;

// Vector4 Explicit instanciations.
template Vector4 operator*(const VectorBase<4, Vector4>&, double);
template Vector4 operator*(double, const VectorBase<4, Vector4>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<4, Vector4>&);
template class VectorBase<4, Vector4>;

}  // namespace math
}  // namespace maliput
