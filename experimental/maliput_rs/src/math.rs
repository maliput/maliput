//! Mathematical types and utilities for maliput.
//!
//! This module provides vector, matrix, quaternion, and rotation types
//! used throughout the maliput API.

use nalgebra::{Matrix3, UnitQuaternion, Vector3 as NVector3};
use std::fmt;
use std::ops::{Add, Mul, Sub};

/// A 3-dimensional vector.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector3 {
    inner: NVector3<f64>,
}

impl Vector3 {
    /// Creates a new Vector3 from x, y, z components.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            inner: NVector3::new(x, y, z),
        }
    }

    /// Returns a zero vector.
    pub fn zero() -> Self {
        Self {
            inner: NVector3::zeros(),
        }
    }

    /// Gets the x component.
    pub fn x(&self) -> f64 {
        self.inner.x
    }

    /// Gets the y component.
    pub fn y(&self) -> f64 {
        self.inner.y
    }

    /// Gets the z component.
    pub fn z(&self) -> f64 {
        self.inner.z
    }

    /// Returns the Euclidean norm (length) of the vector.
    pub fn norm(&self) -> f64 {
        self.inner.norm()
    }

    /// Returns the squared norm of the vector.
    pub fn norm_squared(&self) -> f64 {
        self.inner.norm_squared()
    }

    /// Returns a normalized (unit length) version of the vector.
    ///
    /// Returns None if the vector has zero length.
    pub fn normalize(&self) -> Option<Self> {
        self.inner.try_normalize(1e-15).map(|v| Self { inner: v })
    }

    /// Computes the dot product with another vector.
    pub fn dot(&self, other: &Self) -> f64 {
        self.inner.dot(&other.inner)
    }

    /// Computes the cross product with another vector.
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            inner: self.inner.cross(&other.inner),
        }
    }

    /// Computes the Euclidean distance to another vector.
    pub fn distance(&self, other: &Self) -> f64 {
        (self.inner - other.inner).norm()
    }
}

impl Add for Vector3 {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            inner: self.inner + other.inner,
        }
    }
}

impl Sub for Vector3 {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            inner: self.inner - other.inner,
        }
    }
}

impl Mul<f64> for Vector3 {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self {
        Self {
            inner: self.inner * scalar,
        }
    }
}

impl Mul<Vector3> for f64 {
    type Output = Vector3;

    fn mul(self, vec: Vector3) -> Vector3 {
        Vector3 {
            inner: vec.inner * self,
        }
    }
}

impl fmt::Display for Vector3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {}, {})", self.x(), self.y(), self.z())
    }
}

impl From<NVector3<f64>> for Vector3 {
    fn from(v: NVector3<f64>) -> Self {
        Self { inner: v }
    }
}

impl From<Vector3> for NVector3<f64> {
    fn from(v: Vector3) -> Self {
        v.inner
    }
}

/// Roll-Pitch-Yaw representation of rotation.
///
/// Represents a rotation as a sequence of:
/// 1. Roll around X axis
/// 2. Pitch around Y axis
/// 3. Yaw around Z axis
///
/// All angles are in radians.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RollPitchYaw {
    roll: f64,
    pitch: f64,
    yaw: f64,
}

impl RollPitchYaw {
    /// Creates a new RollPitchYaw from angles in radians.
    pub fn new(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self { roll, pitch, yaw }
    }

    /// Returns the roll angle in radians.
    pub fn roll(&self) -> f64 {
        self.roll
    }

    /// Returns the pitch angle in radians.
    pub fn pitch(&self) -> f64 {
        self.pitch
    }

    /// Returns the yaw angle in radians.
    pub fn yaw(&self) -> f64 {
        self.yaw
    }

    /// Converts to a unit quaternion.
    pub fn to_quaternion(&self) -> UnitQuaternion<f64> {
        UnitQuaternion::from_euler_angles(self.roll, self.pitch, self.yaw)
    }

    /// Creates from a unit quaternion.
    pub fn from_quaternion(q: &UnitQuaternion<f64>) -> Self {
        let (roll, pitch, yaw) = q.euler_angles();
        Self { roll, pitch, yaw }
    }
}

impl fmt::Display for RollPitchYaw {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "(roll: {}, pitch: {}, yaw: {})",
            self.roll, self.pitch, self.yaw
        )
    }
}

/// A 3x3 rotation matrix.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Matrix3x3 {
    inner: Matrix3<f64>,
}

impl Matrix3x3 {
    /// Creates an identity matrix.
    pub fn identity() -> Self {
        Self {
            inner: Matrix3::identity(),
        }
    }

    /// Creates a matrix from row-major elements.
    pub fn from_rows(
        r00: f64,
        r01: f64,
        r02: f64,
        r10: f64,
        r11: f64,
        r12: f64,
        r20: f64,
        r21: f64,
        r22: f64,
    ) -> Self {
        Self {
            inner: Matrix3::new(r00, r01, r02, r10, r11, r12, r20, r21, r22),
        }
    }

    /// Gets the element at (row, col).
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.inner[(row, col)]
    }

    /// Multiplies this matrix by a vector.
    pub fn transform(&self, v: &Vector3) -> Vector3 {
        Vector3::from(self.inner * NVector3::from(*v))
    }
}

impl Default for Matrix3x3 {
    fn default() -> Self {
        Self::identity()
    }
}

impl From<Matrix3<f64>> for Matrix3x3 {
    fn from(m: Matrix3<f64>) -> Self {
        Self { inner: m }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_vector3_basic_operations() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);

        let sum = v1 + v2;
        assert_relative_eq!(sum.x(), 5.0);
        assert_relative_eq!(sum.y(), 7.0);
        assert_relative_eq!(sum.z(), 9.0);

        let diff = v2 - v1;
        assert_relative_eq!(diff.x(), 3.0);
        assert_relative_eq!(diff.y(), 3.0);
        assert_relative_eq!(diff.z(), 3.0);

        let scaled = v1 * 2.0;
        assert_relative_eq!(scaled.x(), 2.0);
        assert_relative_eq!(scaled.y(), 4.0);
        assert_relative_eq!(scaled.z(), 6.0);
    }

    #[test]
    fn test_vector3_dot_cross() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);

        assert_relative_eq!(v1.dot(&v2), 0.0);

        let cross = v1.cross(&v2);
        assert_relative_eq!(cross.x(), 0.0);
        assert_relative_eq!(cross.y(), 0.0);
        assert_relative_eq!(cross.z(), 1.0);
    }

    #[test]
    fn test_roll_pitch_yaw_conversion() {
        let rpy = RollPitchYaw::new(0.1, 0.2, 0.3);
        let q = rpy.to_quaternion();
        let rpy2 = RollPitchYaw::from_quaternion(&q);

        assert_relative_eq!(rpy.roll(), rpy2.roll(), epsilon = 1e-10);
        assert_relative_eq!(rpy.pitch(), rpy2.pitch(), epsilon = 1e-10);
        assert_relative_eq!(rpy.yaw(), rpy2.yaw(), epsilon = 1e-10);
    }
}
