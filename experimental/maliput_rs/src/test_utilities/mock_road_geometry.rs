//! Mock implementation of RoadGeometry for testing.

use std::collections::HashMap;
use std::sync::Arc;

use crate::api::{
    BranchPoint, BranchPointId, IdIndex, InertialPosition, Junction, JunctionId, Lane, LaneId,
    LanePosition, MaliputError, MaliputResult, RoadGeometry, RoadGeometryId, RoadPosition,
    RoadPositionResult, Segment, SegmentId,
};
use crate::math::Vector3;

use super::{MockBranchPoint, MockJunction, MockJunctionBuilder};

/// Mock implementation of the RoadGeometry trait for testing purposes.
#[derive(Debug)]
pub struct MockRoadGeometry {
    id: RoadGeometryId,
    junctions: Vec<Arc<MockJunction>>,
    branch_points: Vec<Arc<MockBranchPoint>>,
    linear_tolerance: f64,
    angular_tolerance: f64,
    scale_length: f64,
    inertial_to_backend_frame_translation: Vector3,
}

impl MockRoadGeometry {
    /// Creates a new MockRoadGeometry with the given ID.
    pub fn new(id: &str) -> Self {
        Self {
            id: RoadGeometryId::new(id.to_string()),
            junctions: Vec::new(),
            branch_points: Vec::new(),
            linear_tolerance: 1e-3,
            angular_tolerance: 1e-3,
            scale_length: 1.0,
            inertial_to_backend_frame_translation: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    /// Adds a junction to this road geometry.
    pub fn add_junction(&mut self, junction: MockJunction) {
        self.junctions.push(Arc::new(junction));
    }

    /// Adds a branch point to this road geometry.
    pub fn add_branch_point(&mut self, branch_point: MockBranchPoint) {
        self.branch_points.push(Arc::new(branch_point));
    }

    /// Sets the linear tolerance.
    pub fn set_linear_tolerance(&mut self, tolerance: f64) {
        self.linear_tolerance = tolerance;
    }

    /// Sets the angular tolerance.
    pub fn set_angular_tolerance(&mut self, tolerance: f64) {
        self.angular_tolerance = tolerance;
    }

    /// Sets the scale length.
    pub fn set_scale_length(&mut self, scale_length: f64) {
        self.scale_length = scale_length;
    }

    /// Sets the inertial to backend frame translation.
    pub fn set_inertial_to_backend_frame_translation(&mut self, translation: Vector3) {
        self.inertial_to_backend_frame_translation = translation;
    }
}

impl RoadGeometry for MockRoadGeometry {
    fn id(&self) -> &RoadGeometryId {
        &self.id
    }

    fn num_junctions(&self) -> usize {
        self.junctions.len()
    }

    fn junction(&self, index: usize) -> MaliputResult<Arc<dyn Junction>> {
        self.junctions
            .get(index)
            .cloned()
            .map(|j| j as Arc<dyn Junction>)
            .ok_or_else(|| MaliputError::IndexOutOfBounds {
                index,
                max: self.junctions.len().saturating_sub(1),
            })
    }

    fn num_branch_points(&self) -> usize {
        self.branch_points.len()
    }

    fn branch_point(&self, index: usize) -> MaliputResult<Arc<dyn BranchPoint>> {
        self.branch_points
            .get(index)
            .cloned()
            .map(|bp| bp as Arc<dyn BranchPoint>)
            .ok_or_else(|| MaliputError::IndexOutOfBounds {
                index,
                max: self.branch_points.len().saturating_sub(1),
            })
    }

    fn by_id(&self) -> Arc<dyn IdIndex> {
        // Create a temporary IdIndex implementation
        Arc::new(MockIdIndex::new(self))
    }

    fn to_road_position(
        &self,
        inertial_position: &InertialPosition,
        _hint: Option<&RoadPosition>,
    ) -> MaliputResult<RoadPositionResult> {
        // Simple mock: find the closest lane position
        let mut best_result: Option<RoadPositionResult> = None;
        let mut best_distance = f64::MAX;

        for junction in &self.junctions {
            for seg_idx in 0..junction.num_segments() {
                if let Ok(segment) = junction.segment(seg_idx) {
                    for lane_idx in 0..segment.num_lanes() {
                        if let Ok(lane) = segment.lane(lane_idx) {
                            if let Ok(result) = lane.to_lane_position(inertial_position) {
                                if result.distance < best_distance {
                                    best_distance = result.distance;
                                    best_result = Some(RoadPositionResult {
                                        road_position: RoadPosition::new(
                                            lane,
                                            result.lane_position.clone(),
                                        ),
                                        nearest_position: result.nearest_position.clone(),
                                        distance: result.distance,
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }

        best_result.ok_or_else(|| MaliputError::IdNotFound("no lane found".to_string()))
    }

    fn find_road_positions(
        &self,
        inertial_position: &InertialPosition,
        radius: f64,
    ) -> MaliputResult<Vec<RoadPositionResult>> {
        if radius < 0.0 {
            return Err(MaliputError::Validation(
                "Radius must be non-negative".to_string(),
            ));
        }

        let mut results = Vec::new();

        for junction in &self.junctions {
            for seg_idx in 0..junction.num_segments() {
                if let Ok(segment) = junction.segment(seg_idx) {
                    for lane_idx in 0..segment.num_lanes() {
                        if let Ok(lane) = segment.lane(lane_idx) {
                            if let Ok(result) = lane.to_lane_position(inertial_position) {
                                if result.distance <= radius {
                                    results.push(RoadPositionResult {
                                        road_position: RoadPosition::new(
                                            lane,
                                            result.lane_position.clone(),
                                        ),
                                        nearest_position: result.nearest_position.clone(),
                                        distance: result.distance,
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }

        Ok(results)
    }

    fn linear_tolerance(&self) -> f64 {
        self.linear_tolerance
    }

    fn angular_tolerance(&self) -> f64 {
        self.angular_tolerance
    }

    fn scale_length(&self) -> f64 {
        self.scale_length
    }

    fn inertial_to_backend_frame_translation(&self) -> Vector3 {
        self.inertial_to_backend_frame_translation.clone()
    }

    fn check_invariants(&self) -> Vec<String> {
        Vec::new()
    }

    fn geo_reference_info(&self) -> Option<String> {
        None
    }
}

/// Helper struct for IdIndex implementation
#[derive(Debug)]
struct MockIdIndex {
    lanes: HashMap<LaneId, Arc<dyn Lane>>,
    segments: HashMap<SegmentId, Arc<dyn Segment>>,
    junctions: HashMap<JunctionId, Arc<dyn Junction>>,
    branch_points: HashMap<BranchPointId, Arc<dyn BranchPoint>>,
}

impl MockIdIndex {
    fn new(rg: &MockRoadGeometry) -> Self {
        let mut lanes = HashMap::new();
        let mut segments = HashMap::new();
        let mut junctions = HashMap::new();
        let branch_points = HashMap::new();

        for junction_arc in &rg.junctions {
            let junction_id = JunctionId::new(junction_arc.id().string().to_string());
            junctions.insert(junction_id, junction_arc.clone() as Arc<dyn Junction>);

            for seg_idx in 0..junction_arc.num_segments() {
                if let Ok(segment) = junction_arc.segment(seg_idx) {
                    let segment_id = SegmentId::new(segment.id().string().to_string());
                    segments.insert(segment_id, segment.clone());

                    for lane_idx in 0..segment.num_lanes() {
                        if let Ok(lane) = segment.lane(lane_idx) {
                            let lane_id = LaneId::new(lane.id().string().to_string());
                            lanes.insert(lane_id, lane.clone());
                        }
                    }
                }
            }
        }

        for bp_arc in &rg.branch_points {
            // TODO: Add branch points if needed
        }

        Self {
            lanes,
            segments,
            junctions,
            branch_points,
        }
    }
}

impl IdIndex for MockIdIndex {
    fn get_lane(&self, id: &LaneId) -> Option<Arc<dyn Lane>> {
        self.lanes.get(id).cloned()
    }

    fn get_lanes(&self) -> HashMap<LaneId, Arc<dyn Lane>> {
        // Create a new HashMap and populate it
        let mut result = HashMap::new();
        for (k, v) in &self.lanes {
            result.insert(LaneId::new(k.string().to_string()), v.clone());
        }
        result
    }

    fn get_segment(&self, id: &SegmentId) -> Option<Arc<dyn Segment>> {
        self.segments.get(id).cloned()
    }

    fn get_junction(&self, id: &JunctionId) -> Option<Arc<dyn Junction>> {
        self.junctions.get(id).cloned()
    }

    fn get_branch_point(&self, id: &BranchPointId) -> Option<Arc<dyn BranchPoint>> {
        self.branch_points.get(id).cloned()
    }
}

/// Builder for creating MockRoadGeometry instances.
pub struct MockRoadGeometryBuilder {
    id: Option<String>,
    linear_tolerance: Option<f64>,
    angular_tolerance: Option<f64>,
    scale_length: Option<f64>,
    inertial_to_backend_frame_translation: Option<Vector3>,
    junction_builders: Vec<Box<dyn FnOnce() -> MockJunction>>,
}

impl Default for MockRoadGeometryBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for MockRoadGeometryBuilder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MockRoadGeometryBuilder")
            .field("id", &self.id)
            .field("linear_tolerance", &self.linear_tolerance)
            .field("angular_tolerance", &self.angular_tolerance)
            .field("scale_length", &self.scale_length)
            .field(
                "inertial_to_backend_frame_translation",
                &self.inertial_to_backend_frame_translation,
            )
            .field("junction_builders_count", &self.junction_builders.len())
            .finish()
    }
}

impl MockRoadGeometryBuilder {
    /// Creates a new MockRoadGeometryBuilder.
    pub fn new() -> Self {
        Self {
            id: None,
            linear_tolerance: None,
            angular_tolerance: None,
            scale_length: None,
            inertial_to_backend_frame_translation: None,
            junction_builders: Vec::new(),
        }
    }

    /// Sets the road geometry ID.
    pub fn id(mut self, id: &str) -> Self {
        self.id = Some(id.to_string());
        self
    }

    /// Sets the linear tolerance.
    pub fn linear_tolerance(mut self, tolerance: f64) -> Self {
        self.linear_tolerance = Some(tolerance);
        self
    }

    /// Sets the angular tolerance.
    pub fn angular_tolerance(mut self, tolerance: f64) -> Self {
        self.angular_tolerance = Some(tolerance);
        self
    }

    /// Sets the scale length.
    pub fn scale_length(mut self, scale_length: f64) -> Self {
        self.scale_length = Some(scale_length);
        self
    }

    /// Sets the inertial to backend frame translation.
    pub fn inertial_to_backend_frame_translation(mut self, translation: Vector3) -> Self {
        self.inertial_to_backend_frame_translation = Some(translation);
        self
    }

    /// Adds a junction using a builder function.
    pub fn add_junction<F>(mut self, f: F) -> Self
    where
        F: FnOnce(MockJunctionBuilder) -> MockJunctionBuilder + 'static,
    {
        self.junction_builders
            .push(Box::new(move || f(MockJunctionBuilder::new()).build()));
        self
    }

    /// Builds the MockRoadGeometry.
    pub fn build(self) -> MockRoadGeometry {
        let id = self.id.unwrap_or_else(|| "mock_road_geometry".to_string());
        let mut rg = MockRoadGeometry::new(&id);

        if let Some(lt) = self.linear_tolerance {
            rg.set_linear_tolerance(lt);
        }
        if let Some(at) = self.angular_tolerance {
            rg.set_angular_tolerance(at);
        }
        if let Some(sl) = self.scale_length {
            rg.set_scale_length(sl);
        }
        if let Some(trans) = self.inertial_to_backend_frame_translation {
            rg.set_inertial_to_backend_frame_translation(trans);
        }

        for builder in self.junction_builders {
            let junction = builder();
            rg.add_junction(junction);
        }

        rg
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::RoadGeometryExt;
    use crate::test_utilities::{MockJunction, MockSegment};

    #[test]
    fn test_mock_road_geometry_basic() {
        let rg = MockRoadGeometry::new("test_rg");
        assert_eq!(rg.id().string(), "test_rg");
        assert_eq!(rg.num_junctions(), 0);
        assert_eq!(rg.num_branch_points(), 0);
    }

    #[test]
    fn test_mock_road_geometry_tolerances() {
        let mut rg = MockRoadGeometry::new("test_rg");
        rg.set_linear_tolerance(0.01);
        rg.set_angular_tolerance(0.001);
        rg.set_scale_length(2.0);

        assert!((rg.linear_tolerance() - 0.01).abs() < 1e-9);
        assert!((rg.angular_tolerance() - 0.001).abs() < 1e-9);
        assert!((rg.scale_length() - 2.0).abs() < 1e-9);
    }

    #[test]
    fn test_mock_road_geometry_with_junctions() {
        let mut rg = MockRoadGeometry::new("test_rg");

        let mut junction = MockJunction::new("junction_0");
        let segment = MockSegment::new("segment_0");
        junction.add_segment(segment);
        rg.add_junction(junction);

        assert_eq!(rg.num_junctions(), 1);
        assert_eq!(rg.junction(0).unwrap().id().string(), "junction_0");
    }

    #[test]
    fn test_mock_road_geometry_iterator() {
        let mut rg = MockRoadGeometry::new("test_rg");

        for i in 0..3 {
            let junction = MockJunction::new(&format!("junction_{}", i));
            rg.add_junction(junction);
        }

        let junction_ids: Vec<_> = rg
            .junctions()
            .map(|j| j.id().string().to_string())
            .collect();
        assert_eq!(junction_ids, vec!["junction_0", "junction_1", "junction_2"]);
    }

    #[test]
    fn test_mock_road_geometry_builder() {
        let rg = MockRoadGeometryBuilder::new()
            .id("builder_rg")
            .linear_tolerance(0.001)
            .angular_tolerance(0.0001)
            .add_junction(|j| {
                j.id("j1").add_segment(|s| {
                    s.id("s1")
                        .add_lane(|l| l.id("l1").length(100.0))
                        .add_lane(|l| l.id("l2").length(100.0))
                })
            })
            .add_junction(|j| {
                j.id("j2")
                    .add_segment(|s| s.id("s2").add_lane(|l| l.id("l3").length(50.0)))
            })
            .build();

        assert_eq!(rg.id().string(), "builder_rg");
        assert!((rg.linear_tolerance() - 0.001).abs() < 1e-9);
        assert!((rg.angular_tolerance() - 0.0001).abs() < 1e-9);
        assert_eq!(rg.num_junctions(), 2);

        // Check junction structure
        let j1 = rg.junction(0).unwrap();
        assert_eq!(j1.id().string(), "j1");
        assert_eq!(j1.num_segments(), 1);

        let s1 = j1.segment(0).unwrap();
        assert_eq!(s1.id().string(), "s1");
        assert_eq!(s1.num_lanes(), 2);
    }
}
