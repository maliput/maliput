//! Core maliput API types and traits.
//!
//! This module defines the primary abstractions for the maliput road network model:
//!
//! - [`RoadGeometry`]: The top-level container for a road network
//! - [`Junction`]: A collection of related segments
//! - [`Segment`]: A bundle of adjacent lanes sharing a road surface
//! - [`Lane`]: A single lane of travel with its own coordinate frame
//! - [`BranchPoint`]: A connectivity node between lane ends
//!
//! ## Design Philosophy
//!
//! The Rust implementation uses traits to define the abstract API, allowing
//! different backends (like OpenDRIVE, procedural generation, etc.) to provide
//! their own implementations.
//!
//! This differs from the C++ implementation which uses abstract base classes
//! with virtual methods (NVI pattern). In Rust, we use:
//!
//! - **Traits** for defining interfaces
//! - **`Arc<dyn Trait>`** for shared ownership where needed
//! - **`&dyn Trait`** for borrowed references to trait objects
//! - **`Result<T, MaliputError>`** for error handling instead of exceptions

mod branch_point;
mod identifiers;
mod junction;
mod lane;
mod lane_data;
mod road_geometry;
mod segment;

pub use branch_point::*;
pub use identifiers::*;
pub use junction::*;
pub use lane::*;
pub use lane_data::*;
pub use road_geometry::{
    IdIndex, RoadGeometry, RoadGeometryBranchPointIterator, RoadGeometryExt,
    RoadGeometryJunctionIterator, RoadPositionResult,
};
pub use segment::*;

use thiserror::Error;

/// Errors that can occur in maliput operations.
#[derive(Error, Debug, Clone)]
pub enum MaliputError {
    /// A validation error occurred (e.g., invalid parameter value).
    #[error("validation error: {0}")]
    Validation(String),

    /// An index was out of bounds.
    #[error("index out of bounds: {index} (max: {max})")]
    IndexOutOfBounds { index: usize, max: usize },

    /// A requested ID was not found.
    #[error("identifier not found: {0}")]
    IdNotFound(String),

    /// A geometric computation failed.
    #[error("geometric error: {0}")]
    Geometric(String),

    /// The operation is not supported by this implementation.
    #[error("not supported: {0}")]
    NotSupported(String),
}

/// Result type for maliput operations.
pub type MaliputResult<T> = Result<T, MaliputError>;

/// Validates a condition and returns an error if it fails.
#[macro_export]
macro_rules! maliput_validate {
    ($condition:expr, $msg:expr) => {
        if !$condition {
            return Err($crate::api::MaliputError::Validation($msg.to_string()));
        }
    };
}
