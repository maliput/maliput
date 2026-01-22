//! Lane trait and related types.
//!
//! A Lane represents a lane of travel in a road network with its own
//! curvilinear coordinate system.

use std::sync::Arc;

use super::{
    BranchPoint, HBounds, InertialPosition, IsoLaneVelocity, LaneEnd, LaneEndWhich, LaneId,
    LanePosition, LanePositionResult, LaneType, MaliputResult, RBounds, Rotation, Segment,
};

/// A Lane represents a lane of travel in a road network.
///
/// A Lane defines a curvilinear coordinate system covering the road surface,
/// with a longitudinal 's' coordinate that expresses the arc-length along a
/// central reference curve. The reference curve nominally represents an ideal
/// travel trajectory along the Lane.
///
/// Lanes are grouped by [`Segment`]. All Lanes belonging to a Segment represent
/// the same road surface, but with different coordinate parameterizations
/// (each Lane has its own reference curve).
///
/// # Coordinate System
///
/// The Lane-frame is a curvilinear coordinate system with:
/// - `s`: longitudinal position along the lane's centerline (0 to length)
/// - `r`: lateral position perpendicular to centerline (+r is left)
/// - `h`: height above the road surface
///
/// # Implementation Notes
///
/// Backend implementations must provide concrete implementations of all
/// trait methods. The coordinate transformations (to/from inertial frame)
/// are the core geometric computations that define the lane's shape.
pub trait Lane: std::fmt::Debug + Send + Sync {
    /// Returns the persistent identifier for this lane.
    fn id(&self) -> &LaneId;

    /// Returns the Segment to which this Lane belongs.
    fn segment(&self) -> Arc<dyn Segment>;

    /// Returns the index of this Lane within its parent Segment.
    ///
    /// Indices increase "to the left" (in the +r direction).
    fn index(&self) -> usize;

    /// Returns a reference to the adjacent Lane to the left of this Lane.
    ///
    /// Left is the +r direction (increasing r coordinate).
    ///
    /// Returns `None` if the parent Segment has no Lane to the left.
    fn to_left(&self) -> Option<Arc<dyn Lane>>;

    /// Returns a reference to the adjacent Lane to the right of this Lane.
    ///
    /// Right is the -r direction (decreasing r coordinate).
    ///
    /// Returns `None` if the parent Segment has no Lane to the right.
    fn to_right(&self) -> Option<Arc<dyn Lane>>;

    /// Returns the arc-length of the Lane along its reference curve.
    ///
    /// This is also the maximum s-coordinate; the domain of s is [0, length()].
    fn length(&self) -> f64;

    /// Returns the nominal lateral (r) bounds for the lane as a function of s.
    ///
    /// These are the lateral bounds for a position considered to be
    /// "staying in the lane".
    ///
    /// # Arguments
    ///
    /// * `s` - The longitudinal position, must be in [0, length()]
    fn lane_bounds(&self, s: f64) -> MaliputResult<RBounds>;

    /// Returns the lateral segment (r) bounds of the lane as a function of s.
    ///
    /// These are the lateral bounds for a position considered to be
    /// "on segment", reflecting the physical extent of the segment's surface.
    ///
    /// # Arguments
    ///
    /// * `s` - The longitudinal position, must be in [0, length()]
    fn segment_bounds(&self, s: f64) -> MaliputResult<RBounds>;

    /// Returns the elevation (h) bounds of the lane as a function of (s, r).
    ///
    /// # Arguments
    ///
    /// * `s` - The longitudinal position, must be in [0, length()]
    /// * `r` - The lateral position, should be within lane_bounds(s)
    fn elevation_bounds(&self, s: f64, r: f64) -> MaliputResult<HBounds>;

    /// Returns the type of this lane (driving, biking, parking, etc.).
    fn lane_type(&self) -> LaneType;

    /// Converts a LanePosition to an InertialPosition.
    ///
    /// Note: There is no constraint on the r coordinate; it can be outside
    /// the lane boundaries. The result represents a point in the s-r plane.
    ///
    /// # Arguments
    ///
    /// * `lane_pos` - The position in lane coordinates. The s component must
    ///                be in [0, length()].
    ///
    /// # Errors
    ///
    /// Returns an error if s is outside [0, length()].
    fn to_inertial_position(&self, lane_pos: &LanePosition) -> MaliputResult<InertialPosition>;

    /// Returns the signed Euclidean curvature at the given position.
    ///
    /// The curvature magnitude is the reciprocal of the osculating circle radius.
    /// - Positive: curves left (toward +r, counter-clockwise from above)
    /// - Negative: curves right (toward -r, clockwise from above)
    ///
    /// # Arguments
    ///
    /// * `lane_pos` - The position at which to compute curvature
    fn get_curvature(&self, lane_pos: &LanePosition) -> MaliputResult<f64>;

    /// Determines the LanePosition corresponding to an InertialPosition.
    ///
    /// The result is constrained to the lane's boundaries.
    ///
    /// # Arguments
    ///
    /// * `inertial_pos` - The position in global coordinates
    fn to_lane_position(&self, inertial_pos: &InertialPosition) -> MaliputResult<LanePositionResult>;

    /// Determines the LanePosition corresponding to an InertialPosition.
    ///
    /// The result is constrained to the segment's boundaries (wider than lane bounds).
    ///
    /// # Arguments
    ///
    /// * `inertial_pos` - The position in global coordinates
    fn to_segment_position(
        &self,
        inertial_pos: &InertialPosition,
    ) -> MaliputResult<LanePositionResult>;

    /// Returns the orientation of the Lane-frame at the given position.
    ///
    /// The rotation expresses the orientation of the Lane-frame basis
    /// with respect to the Inertial-frame basis.
    ///
    /// # Arguments
    ///
    /// * `lane_pos` - The position at which to compute orientation
    fn get_orientation(&self, lane_pos: &LanePosition) -> MaliputResult<Rotation>;

    /// Computes derivatives of LanePosition given a velocity vector.
    ///
    /// # Arguments
    ///
    /// * `position` - The current position
    /// * `velocity` - The velocity in the Lane-frame
    ///
    /// # Returns
    ///
    /// Lane-frame derivatives packed into a LanePosition struct.
    fn eval_motion_derivatives(
        &self,
        position: &LanePosition,
        velocity: &IsoLaneVelocity,
    ) -> MaliputResult<LanePosition>;

    /// Returns the BranchPoint at the specified end of this lane.
    fn get_branch_point(&self, which_end: LaneEndWhich) -> Arc<dyn BranchPoint>;

    /// Returns the set of LaneEnds that connect with this lane on the same
    /// side of the BranchPoint at the specified end.
    ///
    /// This includes this lane itself.
    fn get_confluent_branches(&self, which_end: LaneEndWhich) -> Vec<LaneEnd>;

    /// Returns the set of LaneEnds that continue onward from this lane
    /// at the BranchPoint at the specified end.
    fn get_ongoing_branches(&self, which_end: LaneEndWhich) -> Vec<LaneEnd>;

    /// Returns the default ongoing LaneEnd at the specified end.
    ///
    /// This typically represents "continuing through-traffic" as opposed
    /// to branches executing turns.
    ///
    /// Returns `None` if no default branch has been established.
    fn get_default_branch(&self, which_end: LaneEndWhich) -> Option<LaneEnd>;

    /// Checks if the given LanePosition is contained within this lane's bounds.
    ///
    /// A position is contained if:
    /// - s is in [0, length()]
    /// - r is within lane_bounds(s)
    /// - h is within elevation_bounds(s, r)
    fn contains(&self, lane_position: &LanePosition) -> bool;
}

/// Extension trait for Lane that provides default implementations
/// for some common operations.
pub trait LaneExt: Lane {
    /// Returns whether this lane is the leftmost lane in its segment.
    fn is_leftmost(&self) -> bool {
        self.to_left().is_none()
    }

    /// Returns whether this lane is the rightmost lane in its segment.
    fn is_rightmost(&self) -> bool {
        self.to_right().is_none()
    }

    /// Returns the position at the start of the lane centerline.
    fn start_position(&self) -> LanePosition {
        LanePosition::new(0.0, 0.0, 0.0)
    }

    /// Returns the position at the end of the lane centerline.
    fn end_position(&self) -> LanePosition {
        LanePosition::new(self.length(), 0.0, 0.0)
    }

    /// Returns the InertialPosition at the start of the lane.
    fn start_inertial_position(&self) -> MaliputResult<InertialPosition> {
        self.to_inertial_position(&self.start_position())
    }

    /// Returns the InertialPosition at the end of the lane.
    fn end_inertial_position(&self) -> MaliputResult<InertialPosition> {
        self.to_inertial_position(&self.end_position())
    }
}

// Automatically implement LaneExt for all types that implement Lane
impl<T: Lane + ?Sized> LaneExt for T {}

#[cfg(test)]
mod tests {
    use super::*;

    // These tests would use mock implementations
    // For now, we just verify the module compiles correctly

    #[test]
    fn test_lane_end_which_display() {
        assert_eq!(format!("{}", LaneEndWhich::Start), "Start");
        assert_eq!(format!("{}", LaneEndWhich::Finish), "Finish");
    }
}
