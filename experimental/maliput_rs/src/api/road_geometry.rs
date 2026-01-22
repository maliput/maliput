//! RoadGeometry trait and IdIndex.
//!
//! RoadGeometry is the top-level container for a road network, providing
//! access to junctions, branch points, and coordinate transformations.

use std::collections::HashMap;

use crate::math::Vector3;

use super::{
    BranchPoint, BranchPointId, InertialPosition, Junction, JunctionId, Lane, LaneId,
    LanePosition, MaliputError, MaliputResult, RoadGeometryId, RoadPosition, Segment, SegmentId,
};

/// Result of converting an InertialPosition to a RoadPosition.
#[derive(Debug, Clone)]
pub struct RoadPositionResult<'a> {
    /// The computed RoadPosition.
    pub road_position: RoadPosition<'a>,
    /// The InertialPosition that exactly corresponds to the road position.
    pub nearest_position: InertialPosition,
    /// The Cartesian distance from the query position to nearest_position.
    pub distance: f64,
}

/// Abstract API for the geometry of a road network.
///
/// RoadGeometry provides both the network topology and the geometry of its
/// embedding in 3-space. It is the top-level container for all road network
/// elements.
///
/// # Hierarchy
///
/// ```text
/// RoadGeometry
///   ├── Junction 0
///   │     ├── Segment 0
///   │     │     ├── Lane 0
///   │     │     ├── Lane 1
///   │     │     └── ...
///   │     ├── Segment 1
///   │     └── ...
///   ├── Junction 1
///   │     └── ...
///   ├── BranchPoint 0
///   ├── BranchPoint 1
///   └── ...
/// ```
///
/// # Coordinate Systems
///
/// The RoadGeometry defines the mapping between:
/// - **Inertial Frame**: Global 3D Cartesian coordinates (x, y, z)
/// - **Lane Frame**: Local lane coordinates (s, r, h) for each lane
/// - **Backend Frame**: An optional intermediate frame used by some backends
///
/// # Tolerances
///
/// The geometry defines tolerances that implementations must satisfy:
/// - `linear_tolerance`: Maximum allowed error for position computations
/// - `angular_tolerance`: Maximum allowed error for orientation computations
/// - `scale_length`: Characteristic length scale of the geometry
pub trait RoadGeometry: std::fmt::Debug + Send + Sync {
    /// Returns the persistent identifier for this road geometry.
    fn id(&self) -> &RoadGeometryId;

    /// Returns the number of Junctions in the RoadGeometry.
    fn num_junctions(&self) -> usize;

    /// Returns the Junction at the given index.
    ///
    /// # Arguments
    ///
    /// * `index` - Must be in [0, num_junctions())
    ///
    /// # Errors
    ///
    /// Returns an error if index is out of bounds.
    fn junction(&self, index: usize) -> MaliputResult<&dyn Junction>;

    /// Returns the number of BranchPoints in the RoadGeometry.
    fn num_branch_points(&self) -> usize;

    /// Returns the BranchPoint at the given index.
    ///
    /// # Arguments
    ///
    /// * `index` - Must be in [0, num_branch_points())
    ///
    /// # Errors
    ///
    /// Returns an error if index is out of bounds.
    fn branch_point(&self, index: usize) -> MaliputResult<&dyn BranchPoint>;

    /// Accesses the IdIndex interface for looking up elements by ID.
    fn by_id(&self) -> &dyn IdIndex;

    /// Determines the RoadPosition corresponding to an InertialPosition.
    ///
    /// Returns the point in the RoadGeometry's manifold which is, in the
    /// Inertial-frame, closest to the given position.
    ///
    /// # Arguments
    ///
    /// * `inertial_position` - The position to convert
    /// * `hint` - Optional hint to help determine the result (e.g., nearby position)
    ///
    /// # Guarantees
    ///
    /// The result satisfies:
    /// `result.lane.to_inertial_position(result.pos)` is within `linear_tolerance()`
    /// of `result.nearest_position`.
    fn to_road_position(
        &self,
        inertial_position: &InertialPosition,
        hint: Option<&RoadPosition>,
    ) -> MaliputResult<RoadPositionResult>;

    /// Finds all RoadPositions within a radius of an InertialPosition.
    ///
    /// # Arguments
    ///
    /// * `inertial_position` - The center of the search
    /// * `radius` - The search radius (must be non-negative)
    ///
    /// # Returns
    ///
    /// A vector of RoadPositionResults for lanes whose segment regions
    /// include points within the radius.
    ///
    /// # Errors
    ///
    /// Returns an error if radius is negative.
    fn find_road_positions(
        &self,
        inertial_position: &InertialPosition,
        radius: f64,
    ) -> MaliputResult<Vec<RoadPositionResult>>;

    /// Returns the tolerance guaranteed for linear measurements (positions).
    fn linear_tolerance(&self) -> f64;

    /// Returns the tolerance guaranteed for angular measurements (orientations).
    fn angular_tolerance(&self) -> f64;

    /// Returns the characteristic scale length of this RoadGeometry.
    fn scale_length(&self) -> f64;

    /// Returns the translation vector from Inertial Frame to Backend Frame.
    ///
    /// The Backend Frame is an intermediate inertial frame that differs from
    /// the Inertial Frame by this translation.
    fn inertial_to_backend_frame_translation(&self) -> Vector3;

    /// Verifies invariants guaranteed by the API.
    ///
    /// Returns a vector of strings describing any violations found.
    /// An empty vector indicates success.
    fn check_invariants(&self) -> Vec<String>;

    /// Returns geo-reference information for this RoadGeometry, if available.
    ///
    /// This is typically a WKT (Well-Known Text) projection string.
    fn geo_reference_info(&self) -> Option<String>;
}

/// Interface for accessing road network elements by their unique IDs.
///
/// This provides O(1) or O(log n) access to elements without iterating
/// through the hierarchy.
pub trait IdIndex: std::fmt::Debug + Send + Sync {
    /// Returns the Lane with the given ID, or None if not found.
    fn get_lane(&self, id: &LaneId) -> Option<&dyn Lane>;

    /// Returns all Lanes as a map from ID to Lane reference.
    fn get_lanes(&self) -> HashMap<LaneId, &dyn Lane>;

    /// Returns the Segment with the given ID, or None if not found.
    fn get_segment(&self, id: &SegmentId) -> Option<&dyn Segment>;

    /// Returns the Junction with the given ID, or None if not found.
    fn get_junction(&self, id: &JunctionId) -> Option<&dyn Junction>;

    /// Returns the BranchPoint with the given ID, or None if not found.
    fn get_branch_point(&self, id: &BranchPointId) -> Option<&dyn BranchPoint>;
}

/// Extension trait for RoadGeometry providing additional convenience methods.
pub trait RoadGeometryExt: RoadGeometry {
    /// Returns an iterator over all junctions.
    fn junctions(&self) -> RoadGeometryJunctionIterator<'_>;

    /// Returns an iterator over all branch points.
    fn branch_points(&self) -> RoadGeometryBranchPointIterator<'_>;

    /// Returns the total number of segments across all junctions.
    fn total_segments(&self) -> usize {
        self.junctions().map(|j| j.num_segments()).sum()
    }

    /// Returns the total number of lanes across all segments.
    fn total_lanes(&self) -> usize {
        self.by_id().get_lanes().len()
    }

    /// Samples a lane route at regular intervals and returns InertialPositions.
    ///
    /// # Arguments
    ///
    /// * `lane` - The lane to sample
    /// * `sampling_rate` - The s-coordinate sampling interval
    fn sample_lane(&self, lane: &dyn Lane, sampling_rate: f64) -> MaliputResult<Vec<InertialPosition>> {
        if sampling_rate <= 0.0 {
            return Err(MaliputError::Validation(
                "Sampling rate must be positive".to_string(),
            ));
        }

        let length = lane.length();
        let effective_rate = sampling_rate.max(self.linear_tolerance());

        let mut positions = Vec::new();
        let mut s = 0.0;

        while s < length {
            let lane_pos = LanePosition::new(s, 0.0, 0.0);
            positions.push(lane.to_inertial_position(&lane_pos)?);
            s += effective_rate;
        }

        // Always include the end point
        let end_pos = LanePosition::new(length, 0.0, 0.0);
        positions.push(lane.to_inertial_position(&end_pos)?);

        Ok(positions)
    }
}

// Automatically implement RoadGeometryExt for all types that implement RoadGeometry
impl<T: RoadGeometry> RoadGeometryExt for T {
    fn junctions(&self) -> RoadGeometryJunctionIterator<'_> {
        RoadGeometryJunctionIterator {
            road_geometry: self,
            current: 0,
            total: self.num_junctions(),
        }
    }

    fn branch_points(&self) -> RoadGeometryBranchPointIterator<'_> {
        RoadGeometryBranchPointIterator {
            road_geometry: self,
            current: 0,
            total: self.num_branch_points(),
        }
    }
}

impl RoadGeometryExt for dyn RoadGeometry {
    fn junctions(&self) -> RoadGeometryJunctionIterator<'_> {
        RoadGeometryJunctionIterator {
            road_geometry: self,
            current: 0,
            total: self.num_junctions(),
        }
    }

    fn branch_points(&self) -> RoadGeometryBranchPointIterator<'_> {
        RoadGeometryBranchPointIterator {
            road_geometry: self,
            current: 0,
            total: self.num_branch_points(),
        }
    }
}

/// Iterator over junctions in a road geometry.
pub struct RoadGeometryJunctionIterator<'a> {
    road_geometry: &'a dyn RoadGeometry,
    current: usize,
    total: usize,
}

impl<'a> Iterator for RoadGeometryJunctionIterator<'a> {
    type Item = &'a dyn Junction;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current >= self.total {
            None
        } else {
            let junction = self.road_geometry.junction(self.current).ok()?;
            self.current += 1;
            Some(junction)
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.total - self.current;
        (remaining, Some(remaining))
    }
}

impl<'a> ExactSizeIterator for RoadGeometryJunctionIterator<'a> {}

/// Iterator over branch points in a road geometry.
pub struct RoadGeometryBranchPointIterator<'a> {
    road_geometry: &'a dyn RoadGeometry,
    current: usize,
    total: usize,
}

impl<'a> Iterator for RoadGeometryBranchPointIterator<'a> {
    type Item = &'a dyn BranchPoint;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current >= self.total {
            None
        } else {
            let branch_point = self.road_geometry.branch_point(self.current).ok()?;
            self.current += 1;
            Some(branch_point)
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.total - self.current;
        (remaining, Some(remaining))
    }
}

impl<'a> ExactSizeIterator for RoadGeometryBranchPointIterator<'a> {}

#[cfg(test)]
mod tests {
    #[test]
    fn test_road_geometry_module_compiles() {
        // Placeholder test to verify module compilation
    }
}
