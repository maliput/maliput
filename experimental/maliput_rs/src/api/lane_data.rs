//! Lane-frame data types and coordinate system definitions.
//!
//! This module defines the core data types for positions, rotations, and bounds
//! within the maliput coordinate systems.

use crate::math::{RollPitchYaw, Vector3};
use nalgebra::UnitQuaternion;
use std::fmt;

use super::{Lane, MaliputError, MaliputResult};

/// A position in 3-dimensional geographical Cartesian space (Inertial-frame).
///
/// The Inertial frame is the global coordinate system with components (x, y, z).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct InertialPosition {
    xyz: Vector3,
}

impl InertialPosition {
    /// Creates a new InertialPosition from x, y, z coordinates.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            xyz: Vector3::new(x, y, z),
        }
    }

    /// Creates an InertialPosition from a Vector3.
    pub fn from_xyz(xyz: Vector3) -> Self {
        Self { xyz }
    }

    /// Returns all components as a Vector3.
    pub fn xyz(&self) -> &Vector3 {
        &self.xyz
    }

    /// Returns the x component.
    pub fn x(&self) -> f64 {
        self.xyz.x()
    }

    /// Returns the y component.
    pub fn y(&self) -> f64 {
        self.xyz.y()
    }

    /// Returns the z component.
    pub fn z(&self) -> f64 {
        self.xyz.z()
    }

    /// Returns the Euclidean distance to another position.
    pub fn distance(&self, other: &InertialPosition) -> f64 {
        self.xyz.distance(&other.xyz)
    }

    /// Returns the L2 norm (length from origin).
    pub fn length(&self) -> f64 {
        self.xyz.norm()
    }
}

impl std::ops::Add for InertialPosition {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self::from_xyz(self.xyz + other.xyz)
    }
}

impl std::ops::Sub for InertialPosition {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self::from_xyz(self.xyz - other.xyz)
    }
}

impl std::ops::Mul<f64> for InertialPosition {
    type Output = Self;

    fn mul(self, scalar: f64) -> Self {
        Self::from_xyz(self.xyz * scalar)
    }
}

impl fmt::Display for InertialPosition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "InertialPosition({}, {}, {})", self.x(), self.y(), self.z())
    }
}

/// A 3-dimensional rotation.
///
/// Represents an orientation in 3D space, typically expressing the orientation
/// of a Lane-frame with respect to the Inertial-frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rotation {
    quaternion: UnitQuaternion<f64>,
}

impl Rotation {
    /// Creates an identity rotation (no rotation).
    pub fn identity() -> Self {
        Self {
            quaternion: UnitQuaternion::identity(),
        }
    }

    /// Creates a Rotation from a unit quaternion.
    pub fn from_quaternion(quaternion: UnitQuaternion<f64>) -> Self {
        Self { quaternion }
    }

    /// Creates a Rotation from roll, pitch, yaw angles (in radians).
    ///
    /// The rotation sequence is: roll around X, then pitch around Y, then yaw around Z.
    pub fn from_rpy(roll: f64, pitch: f64, yaw: f64) -> Self {
        Self {
            quaternion: UnitQuaternion::from_euler_angles(roll, pitch, yaw),
        }
    }

    /// Creates a Rotation from a RollPitchYaw.
    pub fn from_roll_pitch_yaw(rpy: &RollPitchYaw) -> Self {
        Self::from_rpy(rpy.roll(), rpy.pitch(), rpy.yaw())
    }

    /// Returns the quaternion representation.
    pub fn quaternion(&self) -> &UnitQuaternion<f64> {
        &self.quaternion
    }

    /// Returns the roll-pitch-yaw representation.
    pub fn rpy(&self) -> RollPitchYaw {
        RollPitchYaw::from_quaternion(&self.quaternion)
    }

    /// Returns the roll angle in radians.
    pub fn roll(&self) -> f64 {
        self.rpy().roll()
    }

    /// Returns the pitch angle in radians.
    pub fn pitch(&self) -> f64 {
        self.rpy().pitch()
    }

    /// Returns the yaw angle in radians.
    pub fn yaw(&self) -> f64 {
        self.rpy().yaw()
    }

    /// Applies this rotation to an InertialPosition.
    pub fn apply(&self, pos: &InertialPosition) -> InertialPosition {
        let v = nalgebra::Vector3::new(pos.x(), pos.y(), pos.z());
        let rotated = self.quaternion * v;
        InertialPosition::new(rotated.x, rotated.y, rotated.z)
    }

    /// Returns the reverse rotation (opposite direction along the lane).
    pub fn reverse(&self) -> Self {
        // Pre-rotation by PI around the vertical axis
        let pi_rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::PI);
        Self {
            quaternion: self.quaternion * pi_rotation,
        }
    }

    /// Computes the angular distance between this rotation and another.
    pub fn distance(&self, other: &Rotation) -> f64 {
        self.quaternion.angle_to(&other.quaternion)
    }
}

impl Default for Rotation {
    fn default() -> Self {
        Self::identity()
    }
}

impl fmt::Display for Rotation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let rpy = self.rpy();
        write!(
            f,
            "Rotation(roll: {:.4}, pitch: {:.4}, yaw: {:.4})",
            rpy.roll(),
            rpy.pitch(),
            rpy.yaw()
        )
    }
}

/// A 3-dimensional position in a Lane-frame.
///
/// Components:
/// - `s`: longitudinal position, as arc-length along the lane's reference curve
/// - `r`: lateral position, perpendicular to the reference line (+r is to the left)
/// - `h`: height above the road surface
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LanePosition {
    srh: Vector3,
}

impl LanePosition {
    /// Creates a new LanePosition from s, r, h coordinates.
    pub fn new(s: f64, r: f64, h: f64) -> Self {
        Self {
            srh: Vector3::new(s, r, h),
        }
    }

    /// Creates a LanePosition from a Vector3 [s, r, h].
    pub fn from_srh(srh: Vector3) -> Self {
        Self { srh }
    }

    /// Returns all components as a Vector3 [s, r, h].
    pub fn srh(&self) -> &Vector3 {
        &self.srh
    }

    /// Returns the s (longitudinal) component.
    pub fn s(&self) -> f64 {
        self.srh.x()
    }

    /// Returns the r (lateral) component.
    pub fn r(&self) -> f64 {
        self.srh.y()
    }

    /// Returns the h (height) component.
    pub fn h(&self) -> f64 {
        self.srh.z()
    }

    /// Sets the s component.
    pub fn set_s(&mut self, s: f64) {
        self.srh = Vector3::new(s, self.r(), self.h());
    }

    /// Sets the r component.
    pub fn set_r(&mut self, r: f64) {
        self.srh = Vector3::new(self.s(), r, self.h());
    }

    /// Sets the h component.
    pub fn set_h(&mut self, h: f64) {
        self.srh = Vector3::new(self.s(), self.r(), h);
    }
}

impl fmt::Display for LanePosition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "LanePosition(s: {}, r: {}, h: {})",
            self.s(),
            self.r(),
            self.h()
        )
    }
}

/// Result of converting an InertialPosition to a LanePosition.
#[derive(Debug, Clone)]
pub struct LanePositionResult {
    /// The computed LanePosition within the lane's or segment's bounds.
    pub lane_position: LanePosition,
    /// The InertialPosition that exactly corresponds to `lane_position`.
    pub nearest_position: InertialPosition,
    /// The Cartesian distance between `nearest_position` and the query position.
    pub distance: f64,
}

/// Isometric velocity vector in a Lane-frame.
///
/// Components (sigma_v, rho_v, eta_v) have the same orientation as (s, r, h)
/// but form an isometric system with a Cartesian distance metric.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct IsoLaneVelocity {
    /// Velocity component along the lane direction.
    pub sigma_v: f64,
    /// Velocity component in the lateral direction.
    pub rho_v: f64,
    /// Velocity component in the vertical direction.
    pub eta_v: f64,
}

impl IsoLaneVelocity {
    /// Creates a new IsoLaneVelocity.
    pub fn new(sigma_v: f64, rho_v: f64, eta_v: f64) -> Self {
        Self {
            sigma_v,
            rho_v,
            eta_v,
        }
    }
}

/// A position in the road network, combining a Lane reference with a LanePosition.
#[derive(Debug, Clone)]
pub struct RoadPosition<'a> {
    /// The lane containing this position.
    pub lane: &'a dyn Lane,
    /// The position within the lane's coordinate frame.
    pub pos: LanePosition,
}

impl<'a> RoadPosition<'a> {
    /// Creates a new RoadPosition.
    pub fn new(lane: &'a dyn Lane, pos: LanePosition) -> Self {
        Self { lane, pos }
    }

    /// Converts this RoadPosition to an InertialPosition.
    pub fn to_inertial_position(&self) -> MaliputResult<InertialPosition> {
        self.lane.to_inertial_position(&self.pos)
    }
}

/// Lateral bounds in the r-component of a Lane-frame.
///
/// The bounds must straddle r = 0 (min <= 0, max >= 0).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct RBounds {
    min: f64,
    max: f64,
}

impl RBounds {
    /// Creates new RBounds.
    ///
    /// # Errors
    ///
    /// Returns an error if min > 0 or max < 0.
    pub fn new(min: f64, max: f64) -> MaliputResult<Self> {
        if min > 0.0 {
            return Err(MaliputError::Validation(format!(
                "RBounds min must be <= 0, got {}",
                min
            )));
        }
        if max < 0.0 {
            return Err(MaliputError::Validation(format!(
                "RBounds max must be >= 0, got {}",
                max
            )));
        }
        Ok(Self { min, max })
    }

    /// Returns the minimum bound (always <= 0).
    pub fn min(&self) -> f64 {
        self.min
    }

    /// Returns the maximum bound (always >= 0).
    pub fn max(&self) -> f64 {
        self.max
    }

    /// Returns the width of the bounds (max - min).
    pub fn width(&self) -> f64 {
        self.max - self.min
    }
}

/// Elevation bounds in the h-component of a Lane-frame.
///
/// The bounds must straddle h = 0 (min <= 0, max >= 0).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct HBounds {
    min: f64,
    max: f64,
}

impl HBounds {
    /// Creates new HBounds.
    ///
    /// # Errors
    ///
    /// Returns an error if min > 0 or max < 0.
    pub fn new(min: f64, max: f64) -> MaliputResult<Self> {
        if min > 0.0 {
            return Err(MaliputError::Validation(format!(
                "HBounds min must be <= 0, got {}",
                min
            )));
        }
        if max < 0.0 {
            return Err(MaliputError::Validation(format!(
                "HBounds max must be >= 0, got {}",
                max
            )));
        }
        Ok(Self { min, max })
    }

    /// Returns the minimum bound (always <= 0).
    pub fn min(&self) -> f64 {
        self.min
    }

    /// Returns the maximum bound (always >= 0).
    pub fn max(&self) -> f64 {
        self.max
    }

    /// Returns the height of the bounds (max - min).
    pub fn height(&self) -> f64 {
        self.max - self.min
    }
}

/// Labels for the endpoints of a Lane.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum LaneEndWhich {
    /// The start of the lane (s == 0).
    Start,
    /// The finish of the lane (s == length).
    Finish,
}

impl fmt::Display for LaneEndWhich {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LaneEndWhich::Start => write!(f, "Start"),
            LaneEndWhich::Finish => write!(f, "Finish"),
        }
    }
}

/// A specific endpoint of a specific Lane.
#[derive(Debug, Clone)]
pub struct LaneEnd<'a> {
    /// The lane.
    pub lane: &'a dyn Lane,
    /// Which end of the lane.
    pub end: LaneEndWhich,
}

impl<'a> LaneEnd<'a> {
    /// Creates a new LaneEnd.
    pub fn new(lane: &'a dyn Lane, end: LaneEndWhich) -> Self {
        Self { lane, end }
    }
}

/// Lane classification options.
///
/// Defines the intended use of a lane.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LaneType {
    /// Unknown or unspecified lane type.
    #[default]
    Unknown,

    // Main Drivable Lanes
    /// Standard driving lane.
    Driving,
    /// Turn lane.
    Turn,
    /// High Occupancy Vehicle lane.
    Hov,
    /// Bus only lane.
    Bus,
    /// Taxi only lane.
    Taxi,
    /// Emergency vehicles only.
    Emergency,

    // Non-Drivable / Special Use
    /// Soft border at the edge of the road.
    Shoulder,
    /// Reserved for cyclists.
    Biking,
    /// Sidewalks / Crosswalks.
    Walking,
    /// Lane with parking spaces.
    Parking,
    /// Hard shoulder / Emergency stop.
    Stop,

    // Infrastructure / Boundaries
    /// Hard border at the edge of the road.
    Border,
    /// Curb stones.
    Curb,
    /// Median between opposing traffic.
    Median,
    /// Generic restricted lane.
    Restricted,
    /// Road works area.
    Construction,
    /// Trains/Trams.
    Rail,

    // Highway / Ramp Semantics
    /// Merge into main road.
    Entry,
    /// Exit from the main road.
    Exit,
    /// Ramp leading to a motorway.
    OnRamp,
    /// Ramp leading away from a motorway.
    OffRamp,
    /// Ramp connecting two motorways.
    ConnectingRamp,
    /// Change roads without driving into the main intersection.
    SlipLane,

    // Abstract / Logical
    /// Intersection crossings with no physical markings.
    Virtual,
}

impl fmt::Display for LaneType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LaneType::Unknown => write!(f, "Unknown"),
            LaneType::Driving => write!(f, "Driving"),
            LaneType::Turn => write!(f, "Turn"),
            LaneType::Hov => write!(f, "HOV"),
            LaneType::Bus => write!(f, "Bus"),
            LaneType::Taxi => write!(f, "Taxi"),
            LaneType::Emergency => write!(f, "Emergency"),
            LaneType::Shoulder => write!(f, "Shoulder"),
            LaneType::Biking => write!(f, "Biking"),
            LaneType::Walking => write!(f, "Walking"),
            LaneType::Parking => write!(f, "Parking"),
            LaneType::Stop => write!(f, "Stop"),
            LaneType::Border => write!(f, "Border"),
            LaneType::Curb => write!(f, "Curb"),
            LaneType::Median => write!(f, "Median"),
            LaneType::Restricted => write!(f, "Restricted"),
            LaneType::Construction => write!(f, "Construction"),
            LaneType::Rail => write!(f, "Rail"),
            LaneType::Entry => write!(f, "Entry"),
            LaneType::Exit => write!(f, "Exit"),
            LaneType::OnRamp => write!(f, "OnRamp"),
            LaneType::OffRamp => write!(f, "OffRamp"),
            LaneType::ConnectingRamp => write!(f, "ConnectingRamp"),
            LaneType::SlipLane => write!(f, "SlipLane"),
            LaneType::Virtual => write!(f, "Virtual"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_inertial_position() {
        let p1 = InertialPosition::new(1.0, 2.0, 3.0);
        let p2 = InertialPosition::new(4.0, 5.0, 6.0);

        assert_relative_eq!(p1.x(), 1.0);
        assert_relative_eq!(p1.y(), 2.0);
        assert_relative_eq!(p1.z(), 3.0);

        let sum = p1 + p2;
        assert_relative_eq!(sum.x(), 5.0);
        assert_relative_eq!(sum.y(), 7.0);
        assert_relative_eq!(sum.z(), 9.0);

        let dist = p1.distance(&p2);
        assert_relative_eq!(dist, (27.0_f64).sqrt()); // sqrt((4-1)^2 + (5-2)^2 + (6-3)^2)
    }

    #[test]
    fn test_lane_position() {
        let pos = LanePosition::new(10.0, 0.5, 0.0);
        assert_relative_eq!(pos.s(), 10.0);
        assert_relative_eq!(pos.r(), 0.5);
        assert_relative_eq!(pos.h(), 0.0);
    }

    #[test]
    fn test_rbounds_validation() {
        // Valid bounds
        let bounds = RBounds::new(-1.5, 1.5).unwrap();
        assert_relative_eq!(bounds.min(), -1.5);
        assert_relative_eq!(bounds.max(), 1.5);
        assert_relative_eq!(bounds.width(), 3.0);

        // Invalid: min > 0
        assert!(RBounds::new(0.1, 1.0).is_err());

        // Invalid: max < 0
        assert!(RBounds::new(-1.0, -0.1).is_err());
    }

    #[test]
    fn test_rotation() {
        let rot = Rotation::from_rpy(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        let pos = InertialPosition::new(1.0, 0.0, 0.0);
        let rotated = rot.apply(&pos);

        // Rotation of 90 degrees around Z should transform (1,0,0) to approximately (0,1,0)
        assert_relative_eq!(rotated.x(), 0.0, epsilon = 1e-10);
        assert_relative_eq!(rotated.y(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(rotated.z(), 0.0, epsilon = 1e-10);
    }
}
