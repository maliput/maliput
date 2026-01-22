//! Mock implementation of Segment for testing.

use crate::api::{Junction, Lane, MaliputError, MaliputResult, Segment, SegmentId};

use super::MockLane;

/// Mock implementation of the Segment trait for testing purposes.
#[derive(Debug)]
pub struct MockSegment {
    id: SegmentId,
    lanes: Vec<MockLane>,
    junction: Option<*const dyn Junction>,
}

// Safety: MockSegment is Send because the raw pointers are only used for
// parent references in a single-threaded test context.
unsafe impl Send for MockSegment {}
unsafe impl Sync for MockSegment {}

impl MockSegment {
    /// Creates a new MockSegment with the given ID.
    pub fn new(id: &str) -> Self {
        Self {
            id: SegmentId::new(id.to_string()),
            lanes: Vec::new(),
            junction: None,
        }
    }

    /// Adds a lane to this segment.
    pub fn add_lane(&mut self, mut lane: MockLane) {
        let index = self.lanes.len();
        lane.set_index(index);
        
        // Set up left/right relationships
        if index > 0 {
            // The new lane has the previous lane to its right
            // (indices increase to the left)
        }
        
        self.lanes.push(lane);
    }

    /// Sets up lane relationships after all lanes are added.
    /// Must be called after add_lane and before using the segment.
    pub fn finalize(&mut self) {
        // Set segment references for all lanes
        let self_ptr: *const dyn Segment = self;
        for lane in &mut self.lanes {
            lane.set_segment(self_ptr);
        }
    }

    /// Sets the parent junction (internal use).
    pub(crate) fn set_junction(&mut self, junction: *const dyn Junction) {
        self.junction = Some(junction);
    }

    /// Returns a mutable reference to a lane by index.
    pub fn lane_mut(&mut self, index: usize) -> Option<&mut MockLane> {
        self.lanes.get_mut(index)
    }
}

impl Segment for MockSegment {
    fn id(&self) -> &SegmentId {
        &self.id
    }

    fn junction(&self) -> &dyn Junction {
        unsafe {
            self.junction
                .map(|p| &*p)
                .expect("MockSegment::junction() called before junction was set")
        }
    }

    fn num_lanes(&self) -> usize {
        self.lanes.len()
    }

    fn lane(&self, index: usize) -> MaliputResult<&dyn Lane> {
        self.lanes.get(index).map(|l| l as &dyn Lane).ok_or_else(|| {
            MaliputError::IndexOutOfBounds {
                index,
                max: self.lanes.len().saturating_sub(1),
            }
        })
    }
}

/// Builder for creating MockSegment instances.
pub struct MockSegmentBuilder {
    id: Option<String>,
    lane_builders: Vec<Box<dyn FnOnce() -> MockLane>>,
}

impl Default for MockSegmentBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for MockSegmentBuilder {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MockSegmentBuilder")
            .field("id", &self.id)
            .field("lane_builders_count", &self.lane_builders.len())
            .finish()
    }
}

impl MockSegmentBuilder {
    /// Creates a new MockSegmentBuilder.
    pub fn new() -> Self {
        Self {
            id: None,
            lane_builders: Vec::new(),
        }
    }

    /// Sets the segment ID.
    pub fn id(mut self, id: &str) -> Self {
        self.id = Some(id.to_string());
        self
    }

    /// Adds a lane using a builder function.
    pub fn add_lane<F>(mut self, f: F) -> Self
    where
        F: FnOnce(super::MockLaneBuilder) -> super::MockLaneBuilder + 'static,
    {
        self.lane_builders
            .push(Box::new(move || f(super::MockLaneBuilder::new()).build()));
        self
    }

    /// Builds the MockSegment.
    pub fn build(self) -> MockSegment {
        let id = self.id.unwrap_or_else(|| "mock_segment".to_string());
        let mut segment = MockSegment::new(&id);

        for builder in self.lane_builders {
            let lane = builder();
            segment.add_lane(lane);
        }

        segment.finalize();
        segment
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::SegmentExt;

    #[test]
    fn test_mock_segment_basic() {
        let mut segment = MockSegment::new("test_segment");
        segment.add_lane(MockLane::new("lane_0").with_length(100.0));
        segment.add_lane(MockLane::new("lane_1").with_length(100.0));
        segment.finalize();

        assert_eq!(segment.id().string(), "test_segment");
        assert_eq!(segment.num_lanes(), 2);
    }

    #[test]
    fn test_mock_segment_lane_access() {
        let mut segment = MockSegment::new("test_segment");
        segment.add_lane(MockLane::new("lane_0"));
        segment.add_lane(MockLane::new("lane_1"));
        segment.finalize();

        let lane0 = segment.lane(0).unwrap();
        assert_eq!(lane0.id().string(), "lane_0");
        assert_eq!(lane0.index(), 0);

        let lane1 = segment.lane(1).unwrap();
        assert_eq!(lane1.id().string(), "lane_1");
        assert_eq!(lane1.index(), 1);
    }

    #[test]
    fn test_mock_segment_lane_out_of_bounds() {
        let mut segment = MockSegment::new("test_segment");
        segment.add_lane(MockLane::new("lane_0"));
        segment.finalize();

        let result = segment.lane(5);
        assert!(result.is_err());
    }

    #[test]
    fn test_mock_segment_iterator() {
        let mut segment = MockSegment::new("test_segment");
        segment.add_lane(MockLane::new("lane_0"));
        segment.add_lane(MockLane::new("lane_1"));
        segment.add_lane(MockLane::new("lane_2"));
        segment.finalize();

        let lane_ids: Vec<_> = segment.lanes().map(|l| l.id().string().to_string()).collect();
        assert_eq!(lane_ids, vec!["lane_0", "lane_1", "lane_2"]);
    }

    #[test]
    fn test_mock_segment_builder() {
        let segment = MockSegmentBuilder::new()
            .id("builder_segment")
            .add_lane(|l| l.id("lane_a").length(50.0))
            .add_lane(|l| l.id("lane_b").length(50.0))
            .build();

        assert_eq!(segment.id().string(), "builder_segment");
        assert_eq!(segment.num_lanes(), 2);
        assert_eq!(segment.lane(0).unwrap().id().string(), "lane_a");
        assert_eq!(segment.lane(1).unwrap().id().string(), "lane_b");
    }
}
