# maliput-rs: Rust Implementation of Maliput API

This is an experimental Rust implementation of the [maliput](https://maliput.readthedocs.io/) road network API.

## Overview

maliput provides a runtime API describing a Road Network model for use in agent and traffic simulations. This Rust implementation aims to provide:

- **Type safety**: Leveraging Rust's type system for safer road network operations
- **Memory safety**: Eliminating common C++ memory issues without garbage collection
- **Performance**: Zero-cost abstractions and efficient memory layout
- **Ergonomics**: Idiomatic Rust API with iterators, Result types, and trait objects

## Architecture Comparison: C++ vs Rust

### C++ Design (Original maliput)

```cpp
// Abstract base class with NVI (Non-Virtual Interface) pattern
class Lane {
public:
    LaneId id() const { return do_id(); }  // Public non-virtual
    // ...
private:
    virtual LaneId do_id() const = 0;      // Private virtual
};

// Concrete implementation
class MalidriveLane : public maliput::geometry_base::Lane {
    LaneId do_id() const override { return id_; }
};
```

### Rust Design

```rust
// Trait defining the interface
pub trait Lane: std::fmt::Debug + Send + Sync {
    fn id(&self) -> &LaneId;
    fn segment(&self) -> &dyn Segment;
    fn to_inertial_position(&self, lane_pos: &LanePosition) -> MaliputResult<InertialPosition>;
    // ...
}

// Concrete implementation
pub struct MalidriveLane {
    id: LaneId,
    // ...
}

impl Lane for MalidriveLane {
    fn id(&self) -> &LaneId { &self.id }
    // ...
}
```

## Key Design Decisions

### 1. Traits Instead of Abstract Classes

Rust doesn't have inheritance, so we use traits to define interfaces:

| C++ Concept | Rust Equivalent |
|-------------|-----------------|
| Abstract base class | `trait` |
| Virtual method | Trait method |
| Pure virtual (`= 0`) | Required trait method |
| `override` | `impl Trait for Type` |
| Multiple inheritance | Multiple trait bounds |

### 2. Error Handling

Instead of C++ exceptions, we use `Result<T, MaliputError>`:

```rust
// C++
InertialPosition ToInertialPosition(const LanePosition&) const;
// May throw maliput::common::assertion_error

// Rust
fn to_inertial_position(&self, lane_pos: &LanePosition) -> MaliputResult<InertialPosition>;
// Returns Err(MaliputError::Validation(...)) on invalid input
```

### 3. Ownership and Borrowing

Rust's ownership system replaces C++ raw pointers with clear lifetime annotations:

```rust
// C++ - Raw pointer, ownership unclear
const Lane* lane = segment->lane(0);

// Rust - Reference with explicit lifetime, segment must outlive lane
let lane: &dyn Lane = segment.lane(0)?;
```

### 4. Type-Safe Identifiers

The `TypeSpecificIdentifier<T>` pattern is preserved using Rust's PhantomData:

```rust
pub struct TypeSpecificIdentifier<T: ?Sized> {
    id: String,
    _marker: PhantomData<T>,  // Compile-time type safety, no runtime cost
}

// These are different types at compile time
type LaneId = TypeSpecificIdentifier<dyn Lane>;
type SegmentId = TypeSpecificIdentifier<dyn Segment>;
```

### 5. Extension Traits

Rust's extension traits provide default implementations without cluttering the main trait:

```rust
// Core trait with required methods
pub trait Lane: Send + Sync {
    fn length(&self) -> f64;
    // ...
}

// Extension trait with default implementations
pub trait LaneExt: Lane {
    fn is_leftmost(&self) -> bool {
        self.to_left().is_none()
    }
    
    fn start_position(&self) -> LanePosition {
        LanePosition::new(0.0, 0.0, 0.0)
    }
}

// Automatically implemented for all Lane types
impl<T: Lane + ?Sized> LaneExt for T {}
```

### 6. Iterators Instead of Index-Based Access

```rust
// Instead of:
for i in 0..road_geometry.num_junctions() {
    let junction = road_geometry.junction(i)?;
}

// Use iterators:
for junction in road_geometry.junctions() {
    // ...
}
```

## Module Structure

```
maliput_rs/
├── Cargo.toml
├── src/
│   ├── lib.rs           # Crate root and documentation
│   ├── math.rs          # Vector3, Rotation, RollPitchYaw
│   └── api/
│       ├── mod.rs           # API module root and MaliputError
│       ├── identifiers.rs   # TypeSpecificIdentifier and ID types
│       ├── lane_data.rs     # InertialPosition, LanePosition, bounds, etc.
│       ├── lane.rs          # Lane trait
│       ├── segment.rs       # Segment trait
│       ├── junction.rs      # Junction trait
│       ├── branch_point.rs  # BranchPoint and LaneEndSet traits
│       └── road_geometry.rs # RoadGeometry and IdIndex traits
```

## Mapping: C++ Types to Rust Types

| C++ Type | Rust Type |
|----------|-----------|
| `maliput::api::InertialPosition` | `InertialPosition` |
| `maliput::api::LanePosition` | `LanePosition` |
| `maliput::api::Rotation` | `Rotation` |
| `maliput::api::RBounds` | `RBounds` |
| `maliput::api::HBounds` | `HBounds` |
| `maliput::api::LaneEnd` | `LaneEnd<'a>` |
| `maliput::api::LaneEnd::Which` | `LaneEndWhich` |
| `maliput::api::LaneType` | `LaneType` |
| `maliput::api::IsoLaneVelocity` | `IsoLaneVelocity` |
| `maliput::api::RoadPosition` | `RoadPosition<'a>` |
| `maliput::api::LanePositionResult` | `LanePositionResult` |
| `maliput::api::RoadPositionResult` | `RoadPositionResult<'a>` |
| `maliput::api::Lane` | `dyn Lane` |
| `maliput::api::Segment` | `dyn Segment` |
| `maliput::api::Junction` | `dyn Junction` |
| `maliput::api::BranchPoint` | `dyn BranchPoint` |
| `maliput::api::RoadGeometry` | `dyn RoadGeometry` |
| `maliput::api::RoadGeometry::IdIndex` | `dyn IdIndex` |
| `maliput::math::Vector3` | `Vector3` |
| `maliput::math::RollPitchYaw` | `RollPitchYaw` |
| `maliput::math::Matrix3` | `Matrix3x3` |

## Thread Safety

All traits require `Send + Sync`, making the API thread-safe by default:

```rust
pub trait Lane: std::fmt::Debug + Send + Sync {
    // ...
}
```

This means implementations must be thread-safe, which Rust enforces at compile time.

## Future Work

### Phase 1: Core API (Current)
- [x] Basic types (Vector3, InertialPosition, LanePosition, etc.)
- [x] Lane trait and data types
- [x] Segment, Junction, BranchPoint traits
- [x] RoadGeometry and IdIndex traits
- [ ] Unit tests with mock implementations

### Phase 2: Rules API
- [ ] Rule types (discrete, range, speed limit)
- [ ] RoadRulebook trait
- [ ] TrafficLights and PhaseRings
- [ ] Intersection modeling

### Phase 3: Backend: maliput_malidrive-rs
- [ ] OpenDRIVE parser (port from C++ or new Rust implementation)
- [ ] Road curve implementations (line, arc, spiral, polynomial)
- [ ] RoadGeometry builder

### Phase 4: Utilities
- [ ] Routing (Router, Route, RoutingConstraints)
- [ ] Mesh generation for visualization
- [ ] KD-tree for spatial queries

### Phase 5: FFI and Python Bindings
- [ ] C FFI for interop with existing maliput backends
- [ ] PyO3 Python bindings

## Building and Testing

```bash
cd experimental/maliput_rs

# Build
cargo build

# Run tests
cargo test

# Generate documentation
cargo doc --open
```

## Dependencies

- **nalgebra**: Linear algebra (vectors, matrices, quaternions)
- **thiserror**: Ergonomic error definitions

## License

BSD 3-Clause License (same as original maliput)

## References

- [maliput Documentation](https://maliput.readthedocs.io/)
- [maliput C++ API](https://github.com/maliput/maliput)
- [maliput_malidrive (OpenDRIVE backend)](https://github.com/maliput/maliput_malidrive)
