//! Junction trait and related types.
//!
//! A Junction is a closed set of Segments with physically coplanar road surfaces.

use std::sync::Arc;

use super::{JunctionId, MaliputResult, RoadGeometry, Segment};

/// A Junction is a closed set of Segments which have physically coplanar
/// road surfaces.
///
/// RoadPositions with the same h value (height above surface) in the domains
/// of two Segments within a Junction map to the same InertialPosition.
/// The Segments need not be directly connected to one another in the
/// network topology.
///
/// Junctions are grouped by [`RoadGeometry`].
///
/// # Use Cases
///
/// - **Simple road sections**: A Junction may contain a single Segment
///   representing a stretch of road.
/// - **Intersections**: A Junction may contain multiple Segments representing
///   the various paths through an intersection.
/// - **Interchanges**: Complex highway interchanges may be modeled as a
///   single Junction with many Segments.
pub trait Junction: std::fmt::Debug + Send + Sync {
    /// Returns the persistent identifier for this junction.
    fn id(&self) -> &JunctionId;

    /// Returns the RoadGeometry to which this Junction belongs.
    fn road_geometry(&self) -> Arc<dyn RoadGeometry>;

    /// Returns the number of Segments in this Junction.
    ///
    /// Return value is non-negative.
    fn num_segments(&self) -> usize;

    /// Returns the Segment at the given index.
    ///
    /// # Arguments
    ///
    /// * `index` - Must be in [0, num_segments())
    ///
    /// # Errors
    ///
    /// Returns an error if index is out of bounds.
    fn segment(&self, index: usize) -> MaliputResult<Arc<dyn Segment>>;
}

/// Extension trait for Junction providing additional convenience methods.
pub trait JunctionExt: Junction {
    /// Returns an iterator over all segments in this junction.
    fn segments(&self) -> JunctionSegmentIterator<'_>;

    /// Returns the total number of lanes across all segments.
    fn total_lanes(&self) -> usize {
        self.segments().map(|s| s.num_lanes()).sum()
    }
}

// Automatically implement JunctionExt for all types that implement Junction
impl<T: Junction> JunctionExt for T {
    fn segments(&self) -> JunctionSegmentIterator<'_> {
        JunctionSegmentIterator {
            junction: self,
            current: 0,
            total: self.num_segments(),
        }
    }
}

impl JunctionExt for dyn Junction {
    fn segments(&self) -> JunctionSegmentIterator<'_> {
        JunctionSegmentIterator {
            junction: self,
            current: 0,
            total: self.num_segments(),
        }
    }
}

/// Iterator over segments in a junction.
pub struct JunctionSegmentIterator<'a> {
    junction: &'a dyn Junction,
    current: usize,
    total: usize,
}

impl<'a> Iterator for JunctionSegmentIterator<'a> {
    type Item = Arc<dyn Segment>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.current >= self.total {
            None
        } else {
            let segment = self.junction.segment(self.current).ok()?;
            self.current += 1;
            Some(segment)
        }
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.total - self.current;
        (remaining, Some(remaining))
    }
}

impl<'a> ExactSizeIterator for JunctionSegmentIterator<'a> {}

#[cfg(test)]
mod tests {
    #[test]
    fn test_junction_module_compiles() {
        // Placeholder test to verify module compilation
    }
}
