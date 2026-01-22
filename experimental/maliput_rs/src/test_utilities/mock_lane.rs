//! Mock implementation of Lane for testing.

use std::sync::Arc;

use crate::api::{
    BranchPoint, HBounds, InertialPosition, IsoLaneVelocity, Lane, LaneEnd, LaneEndWhich, LaneId,
    LanePosition, LanePositionResult, LaneType, MaliputError, MaliputResult, RBounds, Rotation,
    Segment,
};

/// Mock implementation of the Lane trait for testing purposes.
///
/// This mock provides configurable return values for all Lane methods,
/// allowing tests to verify behavior without complex geometric computations.
#[derive(Debug)]
pub struct MockLane {
    id: LaneId,
    index: usize,
    length: f64,
    lane_type: LaneType,
    lane_bounds: RBounds,
    segment_bounds: RBounds,
    elevation_bounds: HBounds,

    // Parent reference (set after construction)
    segment: Option<Arc<dyn Segment>>,

    // Adjacent lane indices (within parent segment)
    left_index: Option<usize>,
    right_index: Option<usize>,

    // Branch point references (set after construction)
    start_branch_point: Option<Arc<dyn BranchPoint>>,
    finish_branch_point: Option<Arc<dyn BranchPoint>>,
}

impl MockLane {
    /// Creates a new MockLane with default values.
    pub fn new(id: &str) -> Self {
        Self {
            id: LaneId::new(id.to_string()),
            index: 0,
            length: 100.0,
            lane_type: LaneType::Driving,
            lane_bounds: RBounds::new(-1.75, 1.75).unwrap(),
            segment_bounds: RBounds::new(-5.0, 5.0).unwrap(),
            elevation_bounds: HBounds::new(-1.0, 5.0).unwrap(),
            segment: None,
            left_index: None,
            right_index: None,
            start_branch_point: None,
            finish_branch_point: None,
        }
    }

    /// Sets the lane index.
    pub fn with_index(mut self, index: usize) -> Self {
        self.index = index;
        self
    }

    /// Sets the lane length.
    pub fn with_length(mut self, length: f64) -> Self {
        self.length = length;
        self
    }

    /// Sets the lane type.
    pub fn with_lane_type(mut self, lane_type: LaneType) -> Self {
        self.lane_type = lane_type;
        self
    }

    /// Sets the lane bounds.
    pub fn with_lane_bounds(mut self, min: f64, max: f64) -> Self {
        self.lane_bounds = RBounds::new(min, max).unwrap();
        self
    }

    /// Sets the segment bounds.
    pub fn with_segment_bounds(mut self, min: f64, max: f64) -> Self {
        self.segment_bounds = RBounds::new(min, max).unwrap();
        self
    }

    /// Sets the elevation bounds.
    pub fn with_elevation_bounds(mut self, min: f64, max: f64) -> Self {
        self.elevation_bounds = HBounds::new(min, max).unwrap();
        self
    }

    /// Sets the left lane index.
    pub fn with_left_index(mut self, index: Option<usize>) -> Self {
        self.left_index = index;
        self
    }

    /// Sets the right lane index.
    pub fn with_right_index(mut self, index: Option<usize>) -> Self {
        self.right_index = index;
        self
    }

    /// Sets the parent segment (internal use).
    pub(crate) fn set_segment(&mut self, segment: Arc<dyn Segment>) {
        self.segment = Some(segment);
    }

    /// Sets the index (internal use).
    pub(crate) fn set_index(&mut self, index: usize) {
        self.index = index;
    }

    /// Sets the start branch point (internal use).
    pub(crate) fn set_start_branch_point(&mut self, bp: Arc<dyn BranchPoint>) {
        self.start_branch_point = Some(bp);
    }

    /// Sets the finish branch point (internal use).
    pub(crate) fn set_finish_branch_point(&mut self, bp: Arc<dyn BranchPoint>) {
        self.finish_branch_point = Some(bp);
    }
}

impl Lane for MockLane {
    fn id(&self) -> &LaneId {
        &self.id
    }

    fn segment(&self) -> Arc<dyn Segment> {
        self.segment
            .clone()
            .expect("MockLane::segment() called before segment was set")
    }

    fn index(&self) -> usize {
        self.index
    }

    fn to_left(&self) -> Option<Arc<dyn Lane>> {
        // In a real implementation, we'd look up the lane in the parent segment.
        // For mocks, we return None and tests should use the segment directly.
        None
    }

    fn to_right(&self) -> Option<Arc<dyn Lane>> {
        // In a real implementation, we'd look up the lane in the parent segment.
        // For mocks, we return None and tests should use the segment directly.
        None
    }

    fn length(&self) -> f64 {
        self.length
    }

    fn lane_bounds(&self, s: f64) -> MaliputResult<RBounds> {
        if s < 0.0 || s > self.length {
            return Err(MaliputError::Validation(format!(
                "s={} is outside lane domain [0, {}]",
                s, self.length
            )));
        }
        Ok(self.lane_bounds)
    }

    fn segment_bounds(&self, s: f64) -> MaliputResult<RBounds> {
        if s < 0.0 || s > self.length {
            return Err(MaliputError::Validation(format!(
                "s={} is outside lane domain [0, {}]",
                s, self.length
            )));
        }
        Ok(self.segment_bounds)
    }

    fn elevation_bounds(&self, s: f64, _r: f64) -> MaliputResult<HBounds> {
        if s < 0.0 || s > self.length {
            return Err(MaliputError::Validation(format!(
                "s={} is outside lane domain [0, {}]",
                s, self.length
            )));
        }
        Ok(self.elevation_bounds)
    }

    fn lane_type(&self) -> LaneType {
        self.lane_type
    }

    fn to_inertial_position(&self, lane_pos: &LanePosition) -> MaliputResult<InertialPosition> {
        let s = lane_pos.s();
        if s < 0.0 || s > self.length {
            return Err(MaliputError::Validation(format!(
                "s={} is outside lane domain [0, {}]",
                s, self.length
            )));
        }
        
        // Simple mock: treat (s, r, h) as (x, y, z) for testing
        // A real implementation would perform actual coordinate transformation
        Ok(InertialPosition::new(s, lane_pos.r(), lane_pos.h()))
    }

    fn get_curvature(&self, lane_pos: &LanePosition) -> MaliputResult<f64> {
        let s = lane_pos.s();
        if s < 0.0 || s > self.length {
            return Err(MaliputError::Validation(format!(
                "s={} is outside lane domain [0, {}]",
                s, self.length
            )));
        }
        // Mock: return zero curvature (straight lane)
        Ok(0.0)
    }

    fn to_lane_position(&self, inertial_pos: &InertialPosition) -> MaliputResult<LanePositionResult> {
        // Simple mock: treat (x, y, z) as (s, r, h)
        let s = inertial_pos.x().clamp(0.0, self.length);
        let r = inertial_pos.y().clamp(self.lane_bounds.min(), self.lane_bounds.max());
        let h = inertial_pos.z();

        let lane_position = LanePosition::new(s, r, h);
        let nearest_position = InertialPosition::new(s, r, h);
        let distance = inertial_pos.distance(&nearest_position);

        Ok(LanePositionResult {
            lane_position,
            nearest_position,
            distance,
        })
    }

    fn to_segment_position(&self, inertial_pos: &InertialPosition) -> MaliputResult<LanePositionResult> {
        // Simple mock: treat (x, y, z) as (s, r, h) with segment bounds
        let s = inertial_pos.x().clamp(0.0, self.length);
        let r = inertial_pos.y().clamp(self.segment_bounds.min(), self.segment_bounds.max());
        let h = inertial_pos.z();

        let lane_position = LanePosition::new(s, r, h);
        let nearest_position = InertialPosition::new(s, r, h);
        let distance = inertial_pos.distance(&nearest_position);

        Ok(LanePositionResult {
            lane_position,
            nearest_position,
            distance,
        })
    }

    fn get_orientation(&self, lane_pos: &LanePosition) -> MaliputResult<Rotation> {
        let s = lane_pos.s();
        if s < 0.0 || s > self.length {
            return Err(MaliputError::Validation(format!(
                "s={} is outside lane domain [0, {}]",
                s, self.length
            )));
        }
        // Mock: return identity rotation (lane aligned with inertial frame)
        Ok(Rotation::identity())
    }

    fn eval_motion_derivatives(
        &self,
        _position: &LanePosition,
        velocity: &IsoLaneVelocity,
    ) -> MaliputResult<LanePosition> {
        // Mock: simple identity mapping for a straight lane
        Ok(LanePosition::new(velocity.sigma_v, velocity.rho_v, velocity.eta_v))
    }

    fn get_branch_point(&self, which_end: LaneEndWhich) -> Arc<dyn BranchPoint> {
        match which_end {
            LaneEndWhich::Start => self
                .start_branch_point
                .clone()
                .expect("MockLane::get_branch_point(Start) called before branch point was set"),
            LaneEndWhich::Finish => self
                .finish_branch_point
                .clone()
                .expect("MockLane::get_branch_point(Finish) called before branch point was set"),
        }
    }

    fn get_confluent_branches(&self, _which_end: LaneEndWhich) -> Vec<LaneEnd> {
        // Mock: return empty for now (would require proper setup)
        Vec::new()
    }

    fn get_ongoing_branches(&self, _which_end: LaneEndWhich) -> Vec<LaneEnd> {
        // Mock: return empty for now (would require proper setup)
        Vec::new()
    }

    fn get_default_branch(&self, _which_end: LaneEndWhich) -> Option<LaneEnd> {
        // Mock: no default branch
        None
    }

    fn contains(&self, lane_position: &LanePosition) -> bool {
        let s = lane_position.s();
        let r = lane_position.r();
        let h = lane_position.h();

        s >= 0.0
            && s <= self.length
            && r >= self.lane_bounds.min()
            && r <= self.lane_bounds.max()
            && h >= self.elevation_bounds.min()
            && h <= self.elevation_bounds.max()
    }
}

/// Builder for creating MockLane instances.
#[derive(Debug, Default)]
pub struct MockLaneBuilder {
    id: Option<String>,
    length: f64,
    lane_type: LaneType,
    lane_bounds_min: f64,
    lane_bounds_max: f64,
}

impl MockLaneBuilder {
    /// Creates a new MockLaneBuilder with default values.
    pub fn new() -> Self {
        Self {
            id: None,
            length: 100.0,
            lane_type: LaneType::Driving,
            lane_bounds_min: -1.75,
            lane_bounds_max: 1.75,
        }
    }

    /// Sets the lane ID.
    pub fn id(mut self, id: &str) -> Self {
        self.id = Some(id.to_string());
        self
    }

    /// Sets the lane length.
    pub fn length(mut self, length: f64) -> Self {
        self.length = length;
        self
    }

    /// Sets the lane type.
    pub fn lane_type(mut self, lane_type: LaneType) -> Self {
        self.lane_type = lane_type;
        self
    }

    /// Sets the lane bounds.
    pub fn lane_bounds(mut self, min: f64, max: f64) -> Self {
        self.lane_bounds_min = min;
        self.lane_bounds_max = max;
        self
    }

    /// Builds the MockLane.
    pub fn build(self) -> MockLane {
        let id = self.id.unwrap_or_else(|| "mock_lane".to_string());
        MockLane::new(&id)
            .with_length(self.length)
            .with_lane_type(self.lane_type)
            .with_lane_bounds(self.lane_bounds_min, self.lane_bounds_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_lane_basic() {
        let lane = MockLane::new("test_lane")
            .with_length(50.0)
            .with_lane_type(LaneType::Biking);

        assert_eq!(lane.id().string(), "test_lane");
        assert_eq!(lane.length(), 50.0);
        assert_eq!(lane.lane_type(), LaneType::Biking);
    }

    #[test]
    fn test_mock_lane_to_inertial_position() {
        let lane = MockLane::new("test_lane").with_length(100.0);

        let lane_pos = LanePosition::new(50.0, 1.0, 0.5);
        let inertial_pos = lane.to_inertial_position(&lane_pos).unwrap();

        // Mock maps (s, r, h) directly to (x, y, z)
        assert_eq!(inertial_pos.x(), 50.0);
        assert_eq!(inertial_pos.y(), 1.0);
        assert_eq!(inertial_pos.z(), 0.5);
    }

    #[test]
    fn test_mock_lane_to_inertial_position_out_of_bounds() {
        let lane = MockLane::new("test_lane").with_length(100.0);

        let lane_pos = LanePosition::new(150.0, 0.0, 0.0); // s > length
        let result = lane.to_inertial_position(&lane_pos);

        assert!(result.is_err());
    }

    #[test]
    fn test_mock_lane_contains() {
        let lane = MockLane::new("test_lane")
            .with_length(100.0)
            .with_lane_bounds(-2.0, 2.0)
            .with_elevation_bounds(-1.0, 5.0);

        // Inside bounds
        assert!(lane.contains(&LanePosition::new(50.0, 0.0, 0.0)));
        assert!(lane.contains(&LanePosition::new(0.0, -2.0, -1.0)));
        assert!(lane.contains(&LanePosition::new(100.0, 2.0, 5.0)));

        // Outside bounds
        assert!(!lane.contains(&LanePosition::new(-1.0, 0.0, 0.0))); // s < 0
        assert!(!lane.contains(&LanePosition::new(101.0, 0.0, 0.0))); // s > length
        assert!(!lane.contains(&LanePosition::new(50.0, 3.0, 0.0))); // r > max
        assert!(!lane.contains(&LanePosition::new(50.0, 0.0, 6.0))); // h > max
    }

    #[test]
    fn test_mock_lane_builder() {
        let lane = MockLaneBuilder::new()
            .id("builder_lane")
            .length(200.0)
            .lane_type(LaneType::Parking)
            .lane_bounds(-3.0, 3.0)
            .build();

        assert_eq!(lane.id().string(), "builder_lane");
        assert_eq!(lane.length(), 200.0);
        assert_eq!(lane.lane_type(), LaneType::Parking);
        
        let bounds = lane.lane_bounds(0.0).unwrap();
        assert_eq!(bounds.min(), -3.0);
        assert_eq!(bounds.max(), 3.0);
    }
}
