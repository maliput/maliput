//! Test utilities and mock implementations for maliput.
//!
//! This module provides mock implementations of the maliput API traits
//! for use in unit tests. These mocks are simple implementations that
//! return configurable values without performing real geometric computations.
//!
//! # Example
//!
//! ```rust
//! use maliput::test_utilities::MockRoadGeometryBuilder;
//!
//! let road_geometry = MockRoadGeometryBuilder::new()
//!     .id("test_rg")
//!     .linear_tolerance(0.01)
//!     .angular_tolerance(0.001)
//!     .add_junction(|j| {
//!         j.id("junction_1")
//!          .add_segment(|s| {
//!              s.id("segment_1")
//!               .add_lane(|l| l.id("lane_1").length(100.0))
//!          })
//!     })
//!     .build();
//! ```

mod mock_branch_point;
mod mock_junction;
mod mock_lane;
mod mock_lane_end_set;
mod mock_road_geometry;
mod mock_segment;

pub use mock_branch_point::*;
pub use mock_junction::*;
pub use mock_lane::*;
pub use mock_lane_end_set::*;
pub use mock_road_geometry::*;
pub use mock_segment::*;
