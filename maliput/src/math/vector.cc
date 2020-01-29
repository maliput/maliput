#include "maliput/math/vector.h"

#include <complex>
#include <functional>
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
  return std::inner_product(vector.values_.cbegin(), vector.values_.cend(), values_.cbegin(), 0);
}

template <std::size_t N, typename Derived>
double VectorBase<N, Derived>::norm() const {
  return std::sqrt(std::inner_product(values_.cbegin(), values_.cend(), values_.cbegin(), 0));
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
  os << "{";
  for (const auto& value : vector.values_) {
    os << value;
    if (*(std::prev(vector.values_.cend())) == value) break;
    os << ", ";
  }
  os << "}";
  return os;
}

Vector2 Vector2::UnitX() { return {1., 0.}; }

Vector2 Vector2::UnitY() { return {0., 1.}; }

Vector3 Vector3::UnitX() { return {1., 0., 0.}; }

Vector3 Vector3::UnitY() { return {0., 1., 0.}; }

Vector3 Vector3::UnitZ() { return {0., 0., 1.}; }

Vector4 Vector4::UnitX() { return {1., 0., 0., 0.}; }

Vector4 Vector4::UnitY() { return {0., 1., 0., 0.}; }

Vector4 Vector4::UnitZ() { return {0., 0., 1., 0.}; }

Vector4 Vector4::UnitW() { return {0., 0., 0., 1.}; }

// Explicit instantiations of commonly used templates instances.
template class VectorBase<2, Vector2>;
template class VectorBase<3, Vector3>;
template class VectorBase<4, Vector4>;
template Vector2 operator*(const VectorBase<2, Vector2>&, double);
template Vector2 operator*(double, const VectorBase<2, Vector2>&);
template Vector3 operator*(const VectorBase<3, Vector3>&, double);
template Vector3 operator*(double, const VectorBase<3, Vector3>&);
template Vector4 operator*(const VectorBase<4, Vector4>&, double);
template Vector4 operator*(double, const VectorBase<4, Vector4>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<2, Vector2>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<3, Vector3>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<4, Vector4>&);

}  // namespace math
}  // namespace maliput
