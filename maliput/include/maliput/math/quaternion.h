#pragma once

#include <cmath>
#include <ostream>

#include "maliput/common/maliput_copyable.h"
#include "maliput/math/matrix.h"
#include "maliput/math/vector.h"

namespace maliput {
namespace math {

/// A Quaternion representation.
class Quaternion {
 public:
  static constexpr double kTolerance{1e-15};

  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Quaternion);

  /// Constructs a Quaternion initialized to \f$ 1+0i+0j+0k \f$.
  Quaternion() : w_(1.), x_(0.), y_(0.), z_(0.) {}

  /// Constructs a Quaternion and initializes it to \f$ w+xi+yj+zk \f$.
  ///
  /// @param w The real \f$ w \f$ coefficient.
  /// @param x The internal \f$ x \f$ coefficient.
  /// @param y The internal \f$ y \f$ coefficient.
  /// @param z The internal \f$ z \f$s coefficient.
  Quaternion(double w, double x, double y, double z) : w_(w), x_(x), y_(y), z_(z) {}

  /// Constructs a Quaternion expressed by `angle` rotation around `axis` vector.
  ///
  /// @param angle A scalar representing an angle in radians.
  /// @param axis A 3D vector representing the rotation angle.
  Quaternion(double angle, const Vector3& axis);

  /// Constructs a Quaternion from a 4D vector matching all its components.
  ///
  /// @param coeffs A 4D vector whose first element is \f$ w \f$, second element
  /// is \f$ x \f$, third element is \f$ y \f$ and fourth element is \f$ z \f$.
  explicit Quaternion(const Vector4& coeffs) : w_(coeffs[0]), x_(coeffs[1]), y_(coeffs[2]), z_(coeffs[3]) {}

  /// Constructs a Quaternion from a 3D rotation matrix.
  ///
  /// @param rotation_matrix A 3x3 matrix whose elements represent a rotation
  /// matrix.
  explicit Quaternion(const Matrix3& rotation_matrix);

  /// \addtogroup constcoeffgettersquaternion Constant coefficient getters of Quaternion.
  /// {@
  double w() const { return w_; }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  /// }@

  /// \addtogroup mutablecoeffgettersquaternion Mutable coefficient getters of Quaternion.
  /// {@
  double& w() { return w_; }
  double& x() { return x_; }
  double& y() { return y_; }
  double& z() { return z_; }
  /// }@

  /// @return This quaternion vector: \f$ [x, y, z] \f$.
  Vector3 vec() const { return Vector3(x_, y_, z_); }

  /// @return This quaternion coefficients: \f$ [w, x, y, z] \f$.
  Vector4 coeffs() const { return Vector4(w_, x_, y_, z_); }

  /// @return A quaternion initialized with \f$ [1, 0, 0, 0] \f$.
  static Quaternion Identity();

  /// @return A quaternion initialized from two vectors.
  /// @see https://bitbucket.org/ignitionrobotics/ign-math/src/default/include/ignition/math/Quaternion.hh
  static Quaternion FromTwoVectors(const Vector3& a, const Vector3& b);

  /// Sets the identity quaternion values as if it was initialized with
  /// \f$ [1, 0, 0, 0] \f$.
  ///
  /// @return `*this`.
  Quaternion& set_identity() {
    w_ = 1.;
    x_ = 0.;
    y_ = 0.;
    z_ = 0.;
    return *this;
  }

  /// @return The squared norm of this quaternion.
  double squared_norm() const { return x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_; }

  /// @return The norm of this quaternion.
  double norm() const { return std::sqrt(x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_); }

  /// Normalizes (`norm()` == 1) this quaternion coefficients.
  ///
  /// When the `norm()` < `kTolerance`, the result is equivalent to call
  /// `set_identity()`.
  void normalize() {
    const double s = norm();
    if (s < kTolerance) {
      set_identity();
    } else {
      w_ /= s;
      x_ /= s;
      y_ /= s;
      z_ /= s;
    }
  }

  /// @return `*this` quaternion with normalized coefficients.
  Quaternion normalized() const {
    Quaternion normalized_q(*this);
    normalized_q.normalize();
    return normalized_q;
  }

  /// Executes the dot product between this quaternion coefficients and `other`.
  ///
  /// @param other A Quaternion.
  /// @return The dot product between this quaternion and `other`.
  double dot(const Quaternion& other) const { return coeffs().dot(other.coeffs()); }

  /// Computes the angular distance between the rotation of this quaternion and
  /// `other`.
  ///
  /// @param other A Quaternion.
  /// @return The angle in radians between `this` rotation and `other`'s
  /// rotation.
  double AngularDistance(const Quaternion& other) const;

  /// @return A rotation Matrix3 equivalent to this normalized quaternion
  /// rotation.
  Matrix3 ToRotationMatrix() const;

  /// Sets this quaternion with the coefficients that makes `a` transform into
  /// `b`.
  ///
  /// @param a A 3D vector.
  /// @param b A 3D vector.
  void SetFromTwoVectors(const Vector3& a, const Vector3& b);

  /// Product operator overload.
  ///
  /// @param q A Quaternion.
  /// @return A quaternion that results from \f$ `this` * `q` \f$.
  Quaternion operator*(const Quaternion& q) const;

  /// Product and assignment operator overload.
  ///
  /// @param q A Quaternion.
  /// @return A mutable reference to `*this` initialized with \f$ `this` * `q` \f$.
  Quaternion& operator*=(const Quaternion& q);

  /// Forwards the call to `TransformVector(v)`.
  Vector3 operator*(const Vector3& v) const;

  /// Operator equal to overload.
  ///
  /// Performs a coefficient wise comparison.
  ///
  /// @param other A quaternion to compare to `this` quaternion.
  /// @return true When `this`' and `other`'s coefficients are equal.
  bool operator==(const Quaternion& other) const;

  /// Operator not equal to overload.
  ///
  /// Performs a coefficient wise comparison.
  ///
  /// @param other A quaternion to compare to `this` quaternion.
  /// @return true When `this`' and `other`'s coefficients are not equal.
  bool operator!=(const Quaternion& other) const { return !operator==(other); }

  /// @return A Quaternion that results from computing the inverse rotation.
  Quaternion Inverse() const;

  /// @return A Quaternion that results from conjugating `this`.
  Quaternion conjugate() const { return Quaternion(w_, -x_, -y_, -z_); }

  /// @return The spherical linear interpolation between the two quaternions
  /// `*this` and `other` at the parameter `t`.
  ///
  /// @param t The interpolation parameter. It must be in [0; 1].
  /// @param other A quaternion.
  Quaternion Slerp(double t, const Quaternion& other) const;

  /// Evaluates if this quaternion is almost equal to `other` up to `precision`
  /// tolerance.
  ///
  /// `this` quaternion is approximately equal to `other` when the squared norm
  /// of the vector difference from both quaternion's coefficients is smaller
  /// than the minimum squared norm between `this` and `other` scaled with the
  /// squared precision.
  ///
  /// @param other A Quaternion to compare with.
  /// @param precision The tolerance.
  /// @return true When this quaternion is almost equal to `other` up to
  /// `precision` tolerance.
  bool IsApprox(const Quaternion& other, double precision) const;

  /// Applies this quaternion transformation to `v` 3D vector.
  ///
  /// @param v A 3D vector.
  /// @return A transformed 3D vector.
  Vector3 TransformVector(const Vector3& v) const;

 private:
  double w_{};
  double x_{};
  double y_{};
  double z_{};
};

/// Serialization operator overload.
///
/// @param os The output stream to serialize `q` into.
/// @param q A Quaternion.
/// @return `os`.
std::ostream& operator<<(std::ostream& os, const Quaternion& q);

}  // namespace math
}  // namespace maliput
