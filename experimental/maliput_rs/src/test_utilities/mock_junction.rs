//! Mock implementation of Junction for testing.

use crate::api::{Junction, JunctionId, MaliputError, MaliputResult, RoadGeometry, Segment};

use super::{MockSegment, MockSegmentBuilder};

/// Mock implementation of the Junction trait for testing purposes.
#[derive(Debug)]
pub struct MockJunction {
    id: JunctionId,
    segments: Vec<MockSegment>,
    road_geometry: Option<*const dyn RoadGeometry>,
}

// Safety: MockJunction is Send because the raw pointers are only used for
// parent references in a single-threaded test context.
unsafe impl Send for MockJunction {}
unsafe impl Sync for MockJunction {}

impl MockJunction {
    /// Creates a new MockJunction with the given ID.
    pub fn new(id: &str) -> Self {
        Self {
            id: JunctionId::new(id.to_string()),
            segments: Vec::new(),
            road_geometry: None,
        }
    }

    /// Adds a segment to this junction.
    pub fn add_segment(&mut self, mut segment: MockSegment) {
        // Set the junction reference
        let self_ptr: *const dyn Junction = self;
        segment.set_junction(self_ptr);
        self.segments.push(segment);
    }

    /// Sets up all segment-junction relationships.
    /// Must be called after adding all segments.
    pub fn finalize(&mut self) {
        let self_ptr: *const dyn Junction = self;
        for segment in &mut self.segments {
            segment.set_junction(self_ptr);
        }
    }

    /// Sets the parent road geometry (internal use).
    pub(crate) fn set_road_geometry(&mut self, rg: *const dyn RoadGeometry) {
        self.road_geometry = Some(rg);
    }

    /// Returns a mutable reference to a segment by index.
    pub fn segment_mut(&mut self, index: usize) -> Option<&mut MockSegment> {
        self.segments.get_mut(index)
    }
}

impl Junction for MockJunction {
    fn id(&self) -> &JunctionId {
        &self.id
    }

    fn road_geometry(&self) -> &dyn RoadGeometry {
        unsafe {
            self.road_geometry
                .map(|p| &*p)
                .expect("MockJunction::road_geometry() called before road_geometry was set")
        }
    }

    fn num_segments(&self) -> usize {
        self.segments.len()
    }

    fn segment(&self, index: usize) -> MaliputResult<&dyn Segment> {
        self.segments
            .get(index)
            .map(|s| s as &dyn Segment)
            .ok_or_else(|| MaliputError::IndexOutOfBounds {
                index,
                max: self.segments.len().saturating_sub(1),
            })
    }
}

/// Builder for creating MockJunction instances.
pub struct MockJunctionBuilder {
    id: Option<String>,
    segment_builders: Vec<Box<dyn FnOnce() -> MockSegment>>,
}

impl Default for MockJunctionBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for MockJunctionBuilder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MockJunctionBuilder")
            .field("id", &self.id)
            .field("segment_builders_count", &self.segment_builders.len())
            .finish()
    }
}

impl MockJunctionBuilder {
    /// Creates a new MockJunctionBuilder.
    pub fn new() -> Self {
        Self {
            id: None,
            segment_builders: Vec::new(),
        }
    }

    /// Sets the junction ID.
    pub fn id(mut self, id: &str) -> Self {
        self.id = Some(id.to_string());
        self
    }

    /// Adds a segment using a builder function.
    pub fn add_segment<F>(mut self, f: F) -> Self
    where
        F: FnOnce(MockSegmentBuilder) -> MockSegmentBuilder + 'static,
    {
        self.segment_builders
            .push(Box::new(move || f(MockSegmentBuilder::new()).build()));
        self
    }

    /// Builds the MockJunction.
    pub fn build(self) -> MockJunction {
        let id = self.id.unwrap_or_else(|| "mock_junction".to_string());
        let mut junction = MockJunction::new(&id);

        for builder in self.segment_builders {
            let segment = builder();
            junction.add_segment(segment);
        }

        junction.finalize();
        junction
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::JunctionExt;

    #[test]
    fn test_mock_junction_basic() {
        let mut junction = MockJunction::new("test_junction");
        
        let mut segment = MockSegment::new("segment_0");
        segment.finalize();
        junction.add_segment(segment);
        
        junction.finalize();

        assert_eq!(junction.id().string(), "test_junction");
        assert_eq!(junction.num_segments(), 1);
    }

    #[test]
    fn test_mock_junction_segment_access() {
        let mut junction = MockJunction::new("test_junction");
        
        let mut segment0 = MockSegment::new("segment_0");
        segment0.finalize();
        junction.add_segment(segment0);
        
        let mut segment1 = MockSegment::new("segment_1");
        segment1.finalize();
        junction.add_segment(segment1);
        
        junction.finalize();

        assert_eq!(junction.segment(0).unwrap().id().string(), "segment_0");
        assert_eq!(junction.segment(1).unwrap().id().string(), "segment_1");
    }

    #[test]
    fn test_mock_junction_segment_out_of_bounds() {
        let junction = MockJunction::new("test_junction");
        let result = junction.segment(0);
        assert!(result.is_err());
    }

    #[test]
    fn test_mock_junction_iterator() {
        let mut junction = MockJunction::new("test_junction");
        
        for i in 0..3 {
            let mut segment = MockSegment::new(&format!("segment_{}", i));
            segment.finalize();
            junction.add_segment(segment);
        }
        junction.finalize();

        let segment_ids: Vec<_> = junction
            .segments()
            .map(|s| s.id().string().to_string())
            .collect();
        assert_eq!(segment_ids, vec!["segment_0", "segment_1", "segment_2"]);
    }

    #[test]
    fn test_mock_junction_builder() {
        let junction = MockJunctionBuilder::new()
            .id("builder_junction")
            .add_segment(|s| {
                s.id("segment_a")
                    .add_lane(|l| l.id("lane_1"))
            })
            .add_segment(|s| {
                s.id("segment_b")
                    .add_lane(|l| l.id("lane_2"))
                    .add_lane(|l| l.id("lane_3"))
            })
            .build();

        assert_eq!(junction.id().string(), "builder_junction");
        assert_eq!(junction.num_segments(), 2);
        
        let seg_a = junction.segment(0).unwrap();
        assert_eq!(seg_a.id().string(), "segment_a");
        assert_eq!(seg_a.num_lanes(), 1);
        
        let seg_b = junction.segment(1).unwrap();
        assert_eq!(seg_b.id().string(), "segment_b");
        assert_eq!(seg_b.num_lanes(), 2);
    }
}
