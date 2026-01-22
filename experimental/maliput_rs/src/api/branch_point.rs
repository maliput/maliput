//! BranchPoint trait and related types.
//!
//! A BranchPoint is a node in the road network where Lanes connect to one another.

use super::{BranchPointId, LaneEnd, MaliputResult, RoadGeometry};

/// A set of LaneEnds.
///
/// This abstraction allows the implementation to decide on storage/indexing
/// strategies (e.g., it could be a view into a database or tiled storage).
pub trait LaneEndSet: std::fmt::Debug + Send + Sync {
    /// Returns the number of LaneEnds in this set.
    fn size(&self) -> usize;

    /// Returns the LaneEnd at the given index.
    ///
    /// # Arguments
    ///
    /// * `index` - Must be in [0, size())
    fn get(&self, index: usize) -> MaliputResult<LaneEnd>;

    /// Returns whether this set is empty.
    fn is_empty(&self) -> bool {
        self.size() == 0
    }
}

/// A BranchPoint is a node in the network of a RoadGeometry at which Lanes
/// connect to one another.
///
/// A BranchPoint is a collection of LaneEnds specifying which Lanes (and which
/// ends of those Lanes) are connected at the BranchPoint.
///
/// # A-Side and B-Side
///
/// LaneEnds participating in a BranchPoint are grouped into two sets,
/// arbitrarily named "A-side" and "B-side":
///
/// - LaneEnds on the **same side** have **coincident** into-the-lane tangent vectors
/// - LaneEnds on **opposite sides** have **anti-parallel** tangent vectors
///
/// ```text
///         A-Side                           B-Side
///    ┌─────────────┐                  ┌─────────────┐
///    │   Lane 1    │ ───────────────► │   Lane 3    │
///    │   (end)     │                  │   (start)   │
///    └─────────────┘                  └─────────────┘
///    ┌─────────────┐                  ┌─────────────┐
///    │   Lane 2    │ ───────────────► │   Lane 4    │
///    │   (end)     │                  │   (start)   │
///    └─────────────┘                  └─────────────┘
/// ```
///
/// In this example:
/// - Lane 1 and Lane 2 ends are on the A-side (they flow into the branch point)
/// - Lane 3 and Lane 4 starts are on the B-side (they flow out of the branch point)
/// - Traffic from Lane 1 or 2 can continue to Lane 3 or 4
pub trait BranchPoint: std::fmt::Debug + Send + Sync {
    /// Returns the persistent identifier for this branch point.
    fn id(&self) -> &BranchPointId;

    /// Returns the RoadGeometry to which this BranchPoint belongs.
    fn road_geometry(&self) -> &dyn RoadGeometry;

    /// Returns the set of LaneEnds on the same side as the given end.
    ///
    /// These are the LaneEnds merging with the given end. The returned set
    /// includes the given end itself.
    ///
    /// # Arguments
    ///
    /// * `end` - Must be connected to this BranchPoint
    fn get_confluent_branches<'a>(&'a self, end: &LaneEnd) -> MaliputResult<&'a dyn LaneEndSet>;

    /// Returns the set of LaneEnds on the opposite side from the given end.
    ///
    /// These are the LaneEnds which the given end flows into.
    ///
    /// # Arguments
    ///
    /// * `end` - Must be connected to this BranchPoint
    fn get_ongoing_branches<'a>(&'a self, end: &LaneEnd) -> MaliputResult<&'a dyn LaneEndSet>;

    /// Returns the default ongoing branch for the given end.
    ///
    /// This typically represents "continuing through-traffic" from the end
    /// (as opposed to a branch executing a turn).
    ///
    /// Returns `None` if no default branch has been established.
    ///
    /// # Arguments
    ///
    /// * `end` - Must be connected to this BranchPoint
    fn get_default_branch(&self, end: &LaneEnd) -> MaliputResult<Option<LaneEnd>>;

    /// Returns the set of LaneEnds on the "A-side".
    fn get_a_side(&self) -> &dyn LaneEndSet;

    /// Returns the set of LaneEnds on the "B-side".
    fn get_b_side(&self) -> &dyn LaneEndSet;
}

/// Extension trait for BranchPoint providing additional convenience methods.
pub trait BranchPointExt: BranchPoint {
    /// Returns the total number of LaneEnds connected to this BranchPoint.
    fn total_connections(&self) -> usize {
        self.get_a_side().size() + self.get_b_side().size()
    }

    /// Returns whether this is a terminal BranchPoint (one side has no connections).
    fn is_terminal(&self) -> bool {
        self.get_a_side().is_empty() || self.get_b_side().is_empty()
    }
}

// Automatically implement BranchPointExt for all types that implement BranchPoint
impl<T: BranchPoint + ?Sized> BranchPointExt for T {}

#[cfg(test)]
mod tests {
    #[test]
    fn test_branch_point_module_compiles() {
        // Placeholder test to verify module compilation
    }
}
