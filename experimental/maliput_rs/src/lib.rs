//! # maliput - A Rust implementation of the maliput road network API
//!
//! This crate provides a Rust implementation of the maliput road network model,
//! which describes road networks for use in agent and traffic simulations.
//!
//! ## Overview
//!
//! maliput provides:
//! - A continuous description of road geometry
//! - Support for dynamic environments with varying rules states
//! - Network topology and 3D spatial embedding of road networks
//!
//! ## Architecture
//!
//! The API is organized around a hierarchical model:
//!
//! ```text
//! RoadGeometry
//!   └── Junction*
//!         └── Segment*
//!               └── Lane*
//!
//! BranchPoint* (connectivity between lane ends)
//! ```
//!
//! ## Coordinate Systems
//!
//! maliput uses two primary coordinate systems:
//!
//! - **Inertial Frame**: Global 3D Cartesian coordinates (x, y, z)
//! - **Lane Frame**: Local lane-relative coordinates (s, r, h)
//!   - `s`: longitudinal position along the lane centerline
//!   - `r`: lateral position perpendicular to centerline (+r is left)
//!   - `h`: height above the road surface
//!
//! ## Usage
//!
//! ```rust,ignore
//! use maliput::api::{RoadGeometry, Lane, InertialPosition, LanePosition};
//!
//! // Load a road network from a backend (e.g., OpenDRIVE via maliput_malidrive)
//! let road_geometry: Box<dyn RoadGeometry> = load_from_backend("map.xodr");
//!
//! // Query the road network
//! let junction = road_geometry.junction(0);
//! let segment = junction.segment(0);
//! let lane = segment.lane(0);
//!
//! // Convert between coordinate systems
//! let lane_pos = LanePosition::new(10.0, 0.5, 0.0);
//! let inertial_pos = lane.to_inertial_position(&lane_pos);
//! ```

pub mod api;
pub mod math;
pub mod test_utilities;

pub use api::*;
pub use math::*;
