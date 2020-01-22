#include "maliput/math/vector.h"

#include <complex>
#include <numeric>

namespace maliput {
namespace math {

template <size_t N, typename Derived>
Derived VectorBase<N, Derived>::Ones() {
  std::array<double, N> ret;
  ret.fill(1);
  return Derived{ret};
}

template <size_t N, typename Derived>
Derived VectorBase<N, Derived>::Zero() {
  std::array<double, N> ret;
  ret.fill(0);
  return Derived{ret};
}

template <size_t N, typename Derived>
VectorBase<N, Derived>::VectorBase() : values_(std::array<double, N>{}) {
  values_.fill(0);
}

template <size_t N, typename Derived>
VectorBase<N, Derived>::VectorBase(std::array<double, N> values) : values_(values) {
  MALIPUT_THROW_UNLESS(values.size() == N);
}

template <size_t N, typename Derived>
VectorBase<N, Derived>::VectorBase(std::initializer_list<double> values) {
  MALIPUT_THROW_UNLESS(values.size() == N);
  std::copy_n(values.begin(), N, values_.begin());
}

template <size_t N, typename Derived>
double VectorBase<N, Derived>::norm() const {
  std::array<double, N> squared;
  std::transform(values_.begin(), values_.end(), squared.begin(), [](double value) { return value * value; });
  return std::sqrt(std::accumulate(squared.begin(), squared.end(), 0));
}

template <size_t N, typename Derived>
void VectorBase<N, Derived>::normalize() {
  const double n = norm();
  std::transform(values_.begin(), values_.end(), values_.begin(), [n](double value) { return value / n; });
}

template <size_t N, typename Derived>
Derived VectorBase<N, Derived>::normalized() const {
  return Derived(*this / norm());
}

template <size_t N, typename Derived>
Derived& VectorBase<N, Derived>::operator=(const VectorBase<N, Derived>& other) {
  if (this != &other) values_ = other.values_;
  return static_cast<Derived&>(*this);
}

template <size_t N, typename Derived>
double VectorBase<N, Derived>::operator[](int index) const {
  if (index < 0 && index >= N) MALIPUT_THROW_MESSAGE("Index out of range");
  return values_[index];
}

template <size_t N, typename Derived>
double& VectorBase<N, Derived>::operator[](int index) {
  if (index < 0 && index >= N) MALIPUT_THROW_MESSAGE("Index out of range");
  return values_[index];
}

template <size_t N, typename Derived>
bool VectorBase<N, Derived>::operator==(const VectorBase<N, Derived>& vector) const {
  return std::equal(values_.cbegin(), values_.cend(), vector.values_.cbegin());
}

template <size_t N, typename Derived>
bool VectorBase<N, Derived>::operator!=(const VectorBase<N, Derived>& vector) const {
  return !(*this == vector);
}

template <size_t N, typename Derived>
Derived VectorBase<N, Derived>::operator+(const VectorBase<N, Derived>& vector) const {
  std::array<double, N> res{};
  std::transform(vector.values_.cbegin(), vector.values_.cend(), values_.cbegin(), res.begin(),
                 [](double a_value, double b_value) { return a_value + b_value; });
  return Derived(res);
}

template <size_t N, typename Derived>
Derived VectorBase<N, Derived>::operator-(const VectorBase<N, Derived>& vector) const {
  std::array<double, N> res{};
  std::transform(vector.values_.cbegin(), vector.values_.cend(), values_.cbegin(), res.begin(),
                 [](double a_value, double b_value) { return a_value - b_value; });
  return Derived(res);
}

template <size_t N, typename Derived>
double VectorBase<N, Derived>::operator*(const VectorBase<N, Derived>& vector) const {
  std::array<double, N> res{};
  std::transform(vector.values_.cbegin(), vector.values_.cend(), values_.cbegin(), res.begin(),
                 [](double a_value, double b_value) { return a_value * b_value; });
  return std::accumulate(res.begin(), res.end(), 0);
}

template <size_t N, typename Derived>
Derived VectorBase<N, Derived>::operator/(double scalar) const {
  std::array<double, N> res{};
  std::transform(values_.cbegin(), values_.cend(), res.begin(), [scalar](double value) { return value / scalar; });
  return {res};
}

template <size_t N_, typename Derived_>
Derived_ operator*(const VectorBase<N_, Derived_>& vector, double scalar) {
  std::array<double, N_> res{};
  std::transform(vector.values_.cbegin(), vector.values_.cend(), res.begin(),
                 [scalar](double value) { return value * scalar; });
  return Derived_(res);
}

template <size_t N_, typename Derived_>
Derived_ operator*(double scalar, const VectorBase<N_, Derived_>& vector) {
  return vector * scalar;
}

template <size_t N_, typename Derived_>
std::ostream& operator<<(std::ostream& os, const VectorBase<N_, Derived_>& vector) {
  os << "{ ";
  for (const auto& value : vector.values_) {
    os << value << " ";
  }
  os << "}";
  return os;
}

Vector2 Vector2::UnitX() {
  std::array<double, 2> ret;
  ret.fill(0);
  ret[0] = 1;
  return ret;
}

Vector2 Vector2::UnitY() {
  std::array<double, 2> ret;
  ret.fill(0);
  ret[1] = 1;
  return ret;
}

Vector3 Vector3::UnitX() {
  std::array<double, 3> ret;
  ret.fill(0);
  ret[0] = 1;
  return ret;
}

Vector3 Vector3::UnitY() {
  std::array<double, 3> ret;
  ret.fill(0);
  ret[1] = 1;
  return ret;
}

Vector3 Vector3::UnitZ() {
  std::array<double, 3> ret;
  ret.fill(0);
  ret[2] = 1;
  return ret;
}

Vector4 Vector4::UnitX() {
  std::array<double, 4> ret;
  ret.fill(0);
  ret[0] = 1;
  return ret;
}

Vector4 Vector4::UnitY() {
  std::array<double, 4> ret;
  ret.fill(0);
  ret[1] = 1;
  return ret;
}

Vector4 Vector4::UnitZ() {
  std::array<double, 4> ret;
  ret.fill(0);
  ret[2] = 1;
  return ret;
}

Vector4 Vector4::UnitW() {
  std::array<double, 4> ret;
  ret.fill(0);
  ret[3] = 1;
  return ret;
}

// Explicit instantiations of commonly used templates instances.
template Vector2 operator*(const VectorBase<2, Vector2>&, double);
template Vector2 operator*(double, const VectorBase<2, Vector2>&);
template Vector3 operator*(const VectorBase<3, Vector3>&, double);
template Vector3 operator*(double, const VectorBase<3, Vector3>&);
template Vector4 operator*(const VectorBase<4, Vector4>&, double);
template Vector4 operator*(double, const VectorBase<4, Vector4>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<2, Vector2>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<3, Vector3>&);
template std::ostream& operator<<(std::ostream&, const VectorBase<4, Vector4>&);
template class VectorBase<2, Vector2>;
template class VectorBase<3, Vector3>;
template class VectorBase<4, Vector4>;

}  // namespace math
}  // namespace maliput
