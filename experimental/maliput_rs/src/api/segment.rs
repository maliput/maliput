//! Segment trait and related types.
//!
//! A Segment represents a bundle of adjacent Lanes sharing a continuously
//! traversable road surface.

use super::{Junction, Lane, MaliputError, MaliputResult, SegmentId};

/// A Segment represents a bundle of adjacent Lanes which share a continuously
/// traversable road surface.
///
/// Every LanePosition on a given Lane of a Segment has a corresponding
/// LanePosition on each other Lane, all with the same height-above-surface h,
/// that all map to the same point in 3-space.
///
/// Segments are grouped by [`Junction`].
///
/// # Lane Indexing
///
/// Lanes within a segment are indexed from 0 to `num_lanes() - 1`.
/// The indexing order is meaningful: numerically adjacent indices correspond
/// to geometrically adjacent Lanes. Indices increase "to the left" (in the
/// direction of increasing r coordinate).
///
/// ```text
///                                            +r direction
///                                          <─────────────
///   Lane N-1  |  Lane N-2  | ... |  Lane 1  |   Lane 0
/// (leftmost)                                 (rightmost)
/// ```
pub trait Segment: std::fmt::Debug + Send + Sync {
    /// Returns the persistent identifier for this segment.
    fn id(&self) -> &SegmentId;

    /// Returns the Junction to which this Segment belongs.
    fn junction(&self) -> &dyn Junction;

    /// Returns the number of Lanes contained in this Segment.
    ///
    /// Return value is non-negative.
    fn num_lanes(&self) -> usize;

    /// Returns the Lane at the given index.
    ///
    /// # Arguments
    ///
    /// * `index` - Must be in [0, num_lanes())
    ///
    /// # Errors
    ///
    /// Returns an error if index is out of bounds.
    fn lane(&self, index: usize) -> MaliputResult<&dyn Lane>;
}

/// Extension trait for Segment providing additional convenience methods.
pub trait SegmentExt: Segment {
    /// Returns an iterator over all lanes in this segment.
    fn lanes(&self) -> SegmentLaneIterator<'_>;

    /// Returns the leftmost lane (highest index).
    fn leftmost_lane(&self) -> MaliputResult<&dyn Lane> {
        let n = self.num_lanes();
        if n == 0 {
            return Err(MaliputError::Validation(
                "Segment has no lanes".to_string(),
            ));
        }
        self.lane(n - 1)
    }

    /// Returns the rightmost lane (index 0).
    fn rightmost_lane(&self) -> MaliputResult<&dyn Lane> {
        if self.num_lanes() == 0 {
            return Err(MaliputError::Validation(
                "Segment has no lanes".to_string(),
            ));
        }
        self.lane(0)
    }
}

// Automatically implement SegmentExt for all types that implement Segment
impl<T: Segment> SegmentExt for T {
    fn lanes(&self) -> SegmentLaneIterator<'_> {
        SegmentLaneIterator {
            segment: self,
            current: 0,
            total: self.num_lanes(),
        }
    }
}

impl SegmentExt for dyn Segment {
    fn lanes(&self) -> SegmentLaneIterator<'_> {
        SegmentLaneIterator {
            segment: self,
            current: 0,
            total: self.num_lanes(),
        }
    }
}

/// Iterator over lanes in a segment.
pub struct SegmentLaneIterator<'a> {
    segment: &'a dyn Segment,
    current: usize,
    total: usize,
}

impl<'a> Iterator for SegmentLaneIterator<'a> {
    type Item = &'a dyn Lane;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current >= self.total {
            None
        } else {
            let lane = self.segment.lane(self.current).ok()?;
            self.current += 1;
            Some(lane)
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.total - self.current;
        (remaining, Some(remaining))
    }
}

impl<'a> ExactSizeIterator for SegmentLaneIterator<'a> {}

#[cfg(test)]
mod tests {
    // Tests would use mock implementations

    #[test]
    fn test_segment_module_compiles() {
        // Placeholder test to verify module compilation
    }
}
