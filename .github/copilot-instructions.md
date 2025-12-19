# GitHub Copilot Onboarding Instructions for maliput

This document provides coding agents with essential information to work efficiently on this repository without extensive exploration.

## Repository Overview

**maliput** is a C++ runtime API describing a Road Network model for use in agent and traffic simulations. It provides:
- A continuous description of road geometry
- Support for dynamic environments with varying rules states
- Network topology and 3D spatial embedding of road networks

**Technologies:** C++17, CMake, Bazel, ROS 2 (ament), Google Test, Eigen3, yaml-cpp
**Documentation:** https://maliput.readthedocs.io/

## Maliput Architecture: API + Backends

**Important:** maliput is primarily an **abstract API** (interfaces and data types). To use maliput in a real application, you need a **backend implementation** that provides concrete road geometry.

### How It Works

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Application                         │
│         (uses maliput::api interfaces)                      │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                     maliput (this repo)                     │
│  - Abstract API (RoadGeometry, Lane, etc.)                  │
│  - Common utilities, math, routing                          │
│  - Plugin system for loading backends                       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                   Backend Implementation                    │
│  (provides concrete RoadGeometry from map data)             │
│                                                             │
│  Options:                                                   │
│  • maliput_malidrive  - OpenDRIVE maps (recommended)        │
│  • maliput_multilane  - Procedural/programmatic roads       │
│  • maliput_dragway    - Simple straight multi-lane roads    │
│  • maliput_osm        - OpenStreetMap data                  │
└─────────────────────────────────────────────────────────────┘
```

### maliput_malidrive (Primary Backend)

[maliput_malidrive](https://github.com/maliput/maliput_malidrive) is the most feature-complete backend, providing maliput implementations from **OpenDRIVE** (`.xodr`) map files.

**Key features:**
- Parses OpenDRIVE 1.4/1.5 format road networks
- Supports complex road geometries (arcs, spirals, polynomials)
- Handles junctions, lane connections, and road rules
- Production-ready for simulation and AV applications

**Usage pattern:**
```cpp
#include <maliput/api/road_network.h>
#include <maliput_malidrive/builder/road_network_builder.h>

// Load an OpenDRIVE map via maliput_malidrive
auto road_network = malidrive::builder::RoadNetworkBuilder(
    {{"opendrive_file", "/path/to/map.xodr"}}
)();

// Now use the maliput API
const maliput::api::RoadGeometry* rg = road_network->road_geometry();
const maliput::api::Lane* lane = rg->junction(0)->segment(0)->lane(0);
```

**Plugin-based loading (preferred):**
```cpp
#include <maliput/plugin/create_road_network.h>

// Load any backend dynamically via plugin system
auto road_network = maliput::plugin::CreateRoadNetwork(
    "maliput_malidrive",  // backend name
    {{"opendrive_file", "/path/to/map.xodr"}}
);
```

### Other Backends

| Backend | Source | Use Case |
|---------|--------|----------|
| **[maliput_malidrive](https://github.com/maliput/maliput_malidrive)** | OpenDRIVE files (`.xodr`) | Production simulations, real-world maps, complex road networks |
| **[maliput_multilane](https://github.com/maliput/maliput_multilane)** | YAML configuration | Procedural roads, integration tests, custom geometries |
| **[maliput_dragway](https://github.com/maliput/maliput_dragway)** | Programmatic parameters | Simple straight multi-lane roads, unit tests, benchmarking |
| **[maliput_osm](https://github.com/maliput/maliput_osm)** | OpenStreetMap data | Large-scale mapping, real-world road data from OSM |

### When Working on maliput

- Changes to `maliput::api` interfaces affect all backends
- Test changes against backends (especially maliput_malidrive) when modifying API
- The `plugin/` component handles dynamic backend loading
- `test_utilities/` provides mocks for testing without a full backend

## Build System & Commands

### Prerequisites

The repository supports two build systems:
1. **CMake with colcon** (ROS 2 workflow) - Primary development workflow
2. **Bazel** - Alternative build system

### CMake/Colcon Build (Recommended)

```bash
# Create workspace
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/maliput/maliput.git

# Install dependencies
export ROS_DISTRO=foxy
rosdep update
rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths src

# Build
colcon build --packages-up-to maliput

# Build with documentation
colcon build --packages-select maliput --cmake-args " -DBUILD_DOCS=On"

# Build with profiler enabled
colcon build --packages-select maliput --cmake-args " -DMALIPUT_PROFILER_ENABLE=On"
```

### Bazel Build

```bash
# Build all targets
bazel build //...

# Build specific library
bazel build //:api
bazel build //:common
bazel build //:math
```

### Testing

```bash
# CMake/colcon
colcon test --packages-select maliput
colcon test-result --verbose

# Bazel
bazel test //...
```

### Code Formatting

```bash
# Check format (CMake builds automatically check via ament_clang_format)
ament_clang_format --config=./.clang-format

# Reformat code
./tools/reformat_code.sh
```

## Project Architecture

### Directory Structure

```
maliput/
├── include/maliput/           # Public headers
│   ├── api/                   # Core road network API
│   ├── base/                  # Base implementations
│   ├── common/                # Common utilities (logging, errors, etc.)
│   ├── drake/                 # Drake-derived utilities
│   ├── geometry_base/         # Geometry base classes
│   ├── math/                  # Math utilities (vectors, matrices, etc.)
│   ├── plugin/                # Plugin system for backends
│   ├── routing/               # Routing algorithms
│   ├── test_utilities/        # Test helpers and mocks
│   └── utility/               # General utilities
├── src/maliput/               # Implementation files
├── test/                      # Unit tests (mirrors include structure)
├── cmake/                     # CMake configuration
├── bazel/                     # Bazel configuration
├── doc/                       # Documentation assets
└── tools/                     # Development scripts
```

### Key Components

| Component | Description |
|-----------|-------------|
| `api` | Core abstractions: `RoadGeometry`, `Lane`, `Segment`, `Junction`, `BranchPoint`, rules |
| `base` | Concrete implementations of API interfaces |
| `common` | Utilities: logging, error handling, copyable macros |
| `geometry_base` | Base classes for implementing road geometry |
| `math` | Linear algebra: `Vector3`, `Matrix3`, `Quaternion`, KD-tree |
| `routing` | Route computation: `Router`, `Route`, `RoutingConstraints` |
| `plugin` | Dynamic loading of maliput backends |
| `utility` | File I/O, mesh generation, thread pools |

### Key Abstractions

The road network model hierarchy:
- **RoadGeometry** - Top-level container for the road network
- **Junction** - Collection of related segments (e.g., an intersection)
- **Segment** - Contiguous group of lanes
- **Lane** - Drivable surface with coordinate frame (s, r, h)
- **BranchPoint** - Connectivity point between lane ends

Coordinate systems:
- **Inertial Frame** - Global 3D coordinates (x, y, z)
- **Lane Frame** - Local lane coordinates (s=along, r=lateral, h=height)

## CI/CD & Validation

### GitHub Workflows

Located in `.github/workflows/`:
- `build.yml` - Main CI: Bazel build + CMake build with coverage
- `build_macos.yaml` - macOS builds
- `sanitizers.yml` - Address/Thread sanitizer builds
- `scan_build.yml` - Static analysis

### Validation Checklist

Before submitting changes:

1. **Format (REQUIRED):**
   ```bash
   ament_clang_format --config=./.clang-format
   # Or use the reformat script:
   ./tools/reformat_code.sh
   ```

2. **Build & Test:**
   ```bash
   colcon build --packages-select maliput
   colcon test --packages-select maliput
   colcon test-result --verbose
   ```

3. **Lint (clang-tidy runs during build with warnings)**

## Code Style Guidelines

### C++ Style

Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) with project-specific configurations.

**Key settings (from `.clang-format`):**
- **Line length:** 120 characters
- **Pointer alignment:** Left (`int* ptr` not `int *ptr`)
- **Include order:** Related header, C headers, C++ headers, library headers, project headers

### Error Handling

Use maliput's error handling macros instead of raw exceptions:

```cpp
// Validate conditions with descriptive messages (PREFERRED)
MALIPUT_VALIDATE(condition, "Descriptive error message");
MALIPUT_VALIDATE(value > 0, "Value must be positive");

// Range validation
MALIPUT_IS_IN_RANGE(value, min_value, max_value);

// Throw with message
MALIPUT_THROW_MESSAGE("Something went wrong");

// Throw unless condition (less preferred - use MALIPUT_VALIDATE instead)
MALIPUT_THROW_UNLESS(condition);
```

### Copy/Move Semantics

Use maliput's macros to explicitly declare copy/move behavior:

```cpp
class MyClass {
 public:
  // Delete all copy/move operations
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MyClass)

  // Or default all operations
  MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MyClass)
};
```

### Logging

Use maliput's logging system:

```cpp
#include "maliput/common/logger.h"

maliput::log()->trace("Trace message");
maliput::log()->debug("Debug message");
maliput::log()->info("Info message");
maliput::log()->warn("Warning message");
maliput::log()->error("Error message");
maliput::log()->critical("Critical message");
```

### Naming Conventions

- **Classes/Structs:** `PascalCase` (e.g., `RoadGeometry`, `LanePosition`)
- **Functions/Methods:** `PascalCase` for public API (e.g., `ToInertialPosition()`)
- **Variables:** `snake_case` (e.g., `lane_position`, `road_geometry`)
- **Member variables:** `snake_case_` with trailing underscore
- **Constants:** `kPascalCase` (e.g., `kDefaultTolerance`)
- **Namespaces:** `lowercase` (e.g., `maliput::api`)

### Header Guards

Use `#pragma once` (not traditional include guards).

### Documentation

Use Doxygen-style comments:

```cpp
/// Brief description of the function.
///
/// Detailed description if needed.
///
/// @param param_name Description of parameter.
/// @returns Description of return value.
/// @throws ExceptionType When this condition occurs.
```

## Testing Patterns

Tests use Google Test (gtest) and Google Mock (gmock):

```cpp
#include <gtest/gtest.h>
#include "maliput/api/lane_data.h"

namespace maliput {
namespace api {
namespace {

GTEST_TEST(LanePositionTest, DefaultConstructor) {
  const LanePosition dut;  // "dut" = Device Under Test
  EXPECT_EQ(dut.s(), 0.);
  EXPECT_EQ(dut.r(), 0.);
  EXPECT_EQ(dut.h(), 0.);
}

}  // namespace
}  // namespace api
}  // namespace maliput
```

Test files are located in `test/` mirroring the `include/maliput/` structure.

## Common Gotchas

1. **Include order matters:** Follow the order enforced by `.clang-format`
2. **Use `MALIPUT_VALIDATE` over `MALIPUT_THROW_UNLESS`:** Prefer descriptive error messages
3. **Namespace wrapping:** Always wrap code in `namespace maliput { ... }`
4. **Virtual destructors:** Abstract classes should have virtual destructors
5. **const correctness:** Use `const` wherever possible

## Related Projects

- [maliput_malidrive](https://github.com/maliput/maliput_malidrive) - OpenDRIVE-based backend
- [maliput_multilane](https://github.com/maliput/maliput_multilane) - Procedural road backend
- [maliput_integration](https://github.com/maliput/maliput_integration) - Integration examples and applications
- [delphyne](https://github.com/maliput/delphyne) - Agent simulation framework using maliput

## Documentation Links

- **Main Documentation:** https://maliput.readthedocs.io/
- **API Reference:** https://maliput.readthedocs.io/en/latest/html/deps/maliput/html/annotated.html
- **Getting Started:** https://maliput.readthedocs.io/en/latest/getting_started.html
- **Contributing:** https://maliput.readthedocs.io/en/latest/contributing.html

## License

BSD 3-Clause License. See [LICENSE](../LICENSE) file.

---

**Trust these instructions.** Only perform additional exploration if information is missing or incorrect.
