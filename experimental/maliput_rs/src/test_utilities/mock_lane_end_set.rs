//! Mock implementation of LaneEndSet for testing.

use crate::api::{LaneEnd, LaneEndSet, MaliputError, MaliputResult};

/// A simple mock implementation of LaneEndSet that stores LaneEnds in a Vec.
#[derive(Debug, Default)]
pub struct MockLaneEndSet {
    /// We store indices to lanes and their ends, to be resolved later.
    /// This is a simplified approach for testing.
    ends: Vec<(usize, crate::api::LaneEndWhich)>,
}

impl MockLaneEndSet {
    /// Creates a new empty MockLaneEndSet.
    pub fn new() -> Self {
        Self { ends: Vec::new() }
    }

    /// Adds a lane end reference (by index and which end).
    pub fn add_end(&mut self, lane_index: usize, which: crate::api::LaneEndWhich) {
        self.ends.push((lane_index, which));
    }

    /// Returns the stored ends for inspection in tests.
    pub fn stored_ends(&self) -> &[(usize, crate::api::LaneEndWhich)] {
        &self.ends
    }
}

impl LaneEndSet for MockLaneEndSet {
    fn size(&self) -> usize {
        self.ends.len()
    }

    fn get(&self, index: usize) -> MaliputResult<LaneEnd> {
        if index >= self.ends.len() {
            return Err(MaliputError::IndexOutOfBounds {
                index,
                max: self.ends.len().saturating_sub(1),
            });
        }
        // In a real implementation, we'd resolve the lane reference here.
        // For testing, we return an error since we can't construct a proper LaneEnd
        // without a reference to the actual lane.
        Err(MaliputError::NotSupported(
            "MockLaneEndSet::get requires lane resolution which is not implemented in mocks. \
             Use stored_ends() for testing instead."
                .to_string(),
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::LaneEndWhich;

    #[test]
    fn test_mock_lane_end_set_empty() {
        let set = MockLaneEndSet::new();
        assert_eq!(set.size(), 0);
        assert!(set.is_empty());
    }

    #[test]
    fn test_mock_lane_end_set_add_ends() {
        let mut set = MockLaneEndSet::new();
        set.add_end(0, LaneEndWhich::Start);
        set.add_end(1, LaneEndWhich::Finish);

        assert_eq!(set.size(), 2);
        assert!(!set.is_empty());

        let ends = set.stored_ends();
        assert_eq!(ends[0], (0, LaneEndWhich::Start));
        assert_eq!(ends[1], (1, LaneEndWhich::Finish));
    }
}
