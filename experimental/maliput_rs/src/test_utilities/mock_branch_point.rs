//! Mock implementation of BranchPoint for testing.

use std::sync::Arc;

use crate::api::{
    BranchPoint, BranchPointId, LaneEnd, LaneEndSet, LaneEndWhich, MaliputResult, RoadGeometry,
};

use super::MockLaneEndSet;

/// Mock implementation of the BranchPoint trait for testing purposes.
#[derive(Debug)]
pub struct MockBranchPoint {
    id: BranchPointId,
    a_side: Arc<MockLaneEndSet>,
    b_side: Arc<MockLaneEndSet>,
    road_geometry: Option<Arc<dyn RoadGeometry>>,
}

impl MockBranchPoint {
    /// Creates a new MockBranchPoint with the given ID.
    pub fn new(id: &str) -> Self {
        Self {
            id: BranchPointId::new(id.to_string()),
            a_side: Arc::new(MockLaneEndSet::new()),
            b_side: Arc::new(MockLaneEndSet::new()),
            road_geometry: None,
        }
    }

    /// Sets the parent road geometry (internal use).
    pub(crate) fn set_road_geometry(&mut self, rg: Arc<dyn RoadGeometry>) {
        self.road_geometry = Some(rg);
    }

    /// Returns a mutable reference to the A-side.
    pub fn a_side_mut(&mut self) -> &mut MockLaneEndSet {
        Arc::get_mut(&mut self.a_side).expect("Cannot get mutable reference to A-side")
    }

    /// Returns a mutable reference to the B-side.
    pub fn b_side_mut(&mut self) -> &mut MockLaneEndSet {
        Arc::get_mut(&mut self.b_side).expect("Cannot get mutable reference to B-side")
    }

    /// Adds a lane end to the A-side by index.
    pub fn add_a_side_end(&mut self, lane_index: usize, end: LaneEndWhich) {
        self.a_side_mut().add_end(lane_index, end);
    }

    /// Adds a lane end to the B-side by index.
    pub fn add_b_side_end(&mut self, lane_index: usize, end: LaneEndWhich) {
        self.b_side_mut().add_end(lane_index, end);
    }
}

impl BranchPoint for MockBranchPoint {
    fn id(&self) -> &BranchPointId {
        &self.id
    }

    fn road_geometry(&self) -> Arc<dyn RoadGeometry> {
        self.road_geometry
            .clone()
            .expect("MockBranchPoint::road_geometry() called before road_geometry was set")
    }

    fn get_a_side(&self) -> Arc<dyn LaneEndSet> {
        self.a_side.clone()
    }

    fn get_b_side(&self) -> Arc<dyn LaneEndSet> {
        self.b_side.clone()
    }

    fn get_default_branch(&self, _end: &LaneEnd) -> MaliputResult<Option<LaneEnd>> {
        // For mock, just return None (no default branch)
        Ok(None)
    }

    fn get_confluent_branches(&self, _end: &LaneEnd) -> MaliputResult<Arc<dyn LaneEndSet>> {
        // For mock, return A side (simplified)
        Ok(self.a_side.clone())
    }

    fn get_ongoing_branches(&self, _end: &LaneEnd) -> MaliputResult<Arc<dyn LaneEndSet>> {
        // For mock, return B side (simplified)
        Ok(self.b_side.clone())
    }
}

/// Builder for creating MockBranchPoint instances.
#[derive(Debug)]
pub struct MockBranchPointBuilder {
    id: Option<String>,
}

impl Default for MockBranchPointBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl MockBranchPointBuilder {
    /// Creates a new MockBranchPointBuilder.
    pub fn new() -> Self {
        Self { id: None }
    }

    /// Sets the branch point ID.
    pub fn id(mut self, id: &str) -> Self {
        self.id = Some(id.to_string());
        self
    }

    /// Builds the MockBranchPoint.
    pub fn build(self) -> MockBranchPoint {
        let id = self.id.unwrap_or_else(|| "mock_branch_point".to_string());
        MockBranchPoint::new(&id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_branch_point_basic() {
        let bp = MockBranchPoint::new("test_bp");
        assert_eq!(bp.id().string(), "test_bp");
        assert_eq!(bp.get_a_side().size(), 0);
        assert_eq!(bp.get_b_side().size(), 0);
    }

    #[test]
    fn test_mock_branch_point_with_ends() {
        let mut bp = MockBranchPoint::new("test_bp");

        // Add lane ends to both sides (using indices)
        bp.add_a_side_end(0, LaneEndWhich::Finish);
        bp.add_b_side_end(1, LaneEndWhich::Start);

        assert_eq!(bp.get_a_side().size(), 1);
        assert_eq!(bp.get_b_side().size(), 1);
    }

    #[test]
    fn test_mock_branch_point_side_counts() {
        let mut bp = MockBranchPoint::new("test_bp");

        bp.add_a_side_end(0, LaneEndWhich::Finish);
        bp.add_a_side_end(1, LaneEndWhich::Finish);
        bp.add_b_side_end(2, LaneEndWhich::Start);

        assert_eq!(bp.get_a_side().size(), 2);
        assert_eq!(bp.get_b_side().size(), 1);
    }

    #[test]
    fn test_mock_branch_point_builder() {
        let bp = MockBranchPointBuilder::new().id("builder_bp").build();

        assert_eq!(bp.id().string(), "builder_bp");
    }
}

