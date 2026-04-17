# RoadObject API

## Overview

`RoadObject` models a static, physical entity in or around the road network that is relevant to simulation but is not part of the road surface itself and is not a traffic signaling device. Examples include barriers, crosswalks, buildings, poles, trees, bollards, fences, and bus shelters.

**Namespace:** `maliput::api::objects`
**Headers:** `maliput/api/objects/road_object.h`, `maliput/api/objects/road_object_book.h`

## Key Design Principles

### Purely Physical / Geometric

A `RoadObject` is a spatial entity with a position, orientation, bounding box, and type. It represents something that physically exists in the road environment — something an agent might need to perceive, avoid, or interact with.

### No Knowledge of Rules

Like `TrafficLight` and `TrafficSign`, a `RoadObject` has **no reference to any Rule**. If a road object implies a rule (e.g., a crosswalk implies pedestrian right-of-way), the rule references the object through `Rule::State::related_unique_ids` — not the other way around. The convention is to use the group name `"Road Object"` with `UniqueId` values constructed from the `RoadObject`'s string ID.

### Separate from Traffic Devices

`RoadObject` lives in `maliput::api::objects` (not `maliput::api::rules`) because these objects are not traffic control devices. They are environmental features of the road network. This is in contrast to `TrafficLight` and `TrafficSign`, which live in `maliput::api::rules` because they are intrinsically tied to traffic regulation.

### Extensible and Subclassable

- The `RoadObject` class is **non-final** with a protected constructor, allowing backend implementations to subclass it for custom storage or behavior.
- The `properties` map (`unordered_map<string, string>`) allows backends to attach arbitrary key-value metadata without API changes.
- The `subtype` string provides fine-grained classification within a `RoadObjectType` category.

## Supporting Types

### RoadObjectType (enum)

Type-safe enumeration of common object categories:

| Enum Value | Description |
|------------|-------------|
| `kUnknown` | Unknown or unclassified object |
| `kBarrier` | Continuous roadside barrier (guard rail, wall, fence) |
| `kBuilding` | Building or permanent structure |
| `kCrosswalk` | Pedestrian or bicycle crossing |
| `kGantry` | Overhead structure for mounting signals |
| `kObstacle` | Static obstacle that cannot be passed |
| `kParkingSpace` | Designated parking area |
| `kPole` | Vertical pole structure (street lamp, sign pole) |
| `kRoadMark` | Painted road marking (arrows, symbols, text) |
| `kRoadSurface` | Road surface element (manhole, speed bump) |
| `kStopLine` | Stop line painted on the road surface |
| `kTrafficIsland` | Traffic island or median |
| `kTree` | Individual tree |
| `kVegetation` | Vegetation area (bush, forest, hedge) |

The `subtype` string provides additional classification within a type (e.g., a `kBarrier` with subtype `"guardRail"` vs `"wall"`).

### BoundingBox (`maliput::math::BoundingBox`)

`RoadObject` reuses the existing `maliput::math::BoundingBox` class, which provides:

- **Position** — centroid of the bounding box in the inertial frame.
- **Dimensions** — `box_size()` returns a `Vector3(length, width, height)`.
- **Orientation** — `RollPitchYaw` orientation in the inertial frame.
- **Spatial queries** — `Contains(position)`, `Overlaps(other)`, `IsBoxContained(other)`, `IsBoxIntersected(other)`.
- **Vertices** — `get_vertices()` returns the 8 corners in inertial coordinates.

For cylindrical objects (poles, trees), backends can approximate with a square box where `length = width = 2 * radius`.

### Outline / OutlineCorner

For precise geometry beyond a bounding box approximation:

- **`Outline`** — ordered sequence of `OutlineCorner` points, closed (polygon) or open (polyline).
- **`OutlineCorner`** — a vertex position in the inertial frame (x, y, z), with an optional per-corner extrusion height for variable-height objects (e.g., tapered walls).
- When outlines are defined on a `RoadObject`, they **supersede** the bounding box for precise geometry.

### RoadObjectPosition

Dual positioning system:

- **Inertial position** (always available) — global (x, y, z) coordinates.
- **Lane-relative position** (optional) — `LaneId` + `LanePosition(s, r, h)` for objects semantically associated with a specific lane (e.g., a parking space alongside a lane).

## RoadObject

| Property | Type | Description |
|----------|------|-------------|
| `id` | `RoadObject::Id` | Unique identifier within the `RoadObjectBook` |
| `name` | `optional<string>` | Human-readable name for debugging/display |
| `type` | `RoadObjectType` | Object category (enum) |
| `subtype` | `optional<string>` | Fine-grained classification within a type |
| `position` | `RoadObjectPosition` | Inertial + optional lane-relative position |
| `orientation` | `Rotation` | Orientation in the inertial frame |
| `bounding_box` | `maliput::math::BoundingBox` | Axis-aligned or oriented bounding volume |
| `is_dynamic` | `bool` | Whether object has movable parts |
| `related_lanes` | `vector<LaneId>` | All lanes this object is spatially relevant to |
| `outlines` | `vector<Outline>` | Detailed shape geometry (supersedes bounding box) |
| `properties` | `unordered_map<string, string>` | Backend-specific key-value metadata |

### Lane Association

`RoadObject` provides two complementary lane association mechanisms:

1. **`RoadObjectPosition::lane_id()`** — the single primary lane the object is *positioned on* (e.g., a speed bump on lane X).
2. **`related_lanes()`** — *all* lanes the object is spatially relevant to (e.g., a crosswalk spanning lanes X, Y, and Z).

Both are spatial facts, not rule relationships.

## RoadObjectBook

`RoadObjectBook` is the abstract registry interface for road objects. It provides the NVI pattern used throughout maliput.

| Method | Returns | Description |
|--------|---------|-------------|
| `RoadObjects()` | `vector<const RoadObject*>` | All objects in the book |
| `GetRoadObject(id)` | `const RoadObject*` | A specific object by ID, or `nullptr` |
| `FindByType(type)` | `vector<const RoadObject*>` | All objects of a given `RoadObjectType` |
| `FindByLane(lane_id)` | `vector<const RoadObject*>` | All objects relevant to the given lane |
| `FindInRadius(pos, radius)` | `vector<const RoadObject*>` | All objects within `radius` meters of `pos` |

Backend implementations populate the book from their data sources (e.g., OpenDRIVE `<objects>` elements, OSM features).

## See Also

- [regulatory_elements_and_objects.md](regulatory_elements_and_objects.md) — comparison of `RoadObject`, `TrafficSign`, and `TrafficLight` APIs.

## Example

```cpp
#include "maliput/api/objects/road_object.h"
#include "maliput/api/objects/road_object_book.h"

using namespace maliput::api::objects;

// Access road objects
const auto* book = road_network->road_object_book();

// Get all barriers
auto barriers = book->FindByType(RoadObjectType::kBarrier);

// Get all objects near a specific lane
auto objects_for_lane = book->FindByLane(LaneId("lane_7"));

// Find objects within 50 meters of a position
auto nearby = book->FindInRadius(InertialPosition(100., 200., 0.), 50.0);

// Inspect a specific object
const auto* obj = book->GetRoadObject(RoadObject::Id("guardrail_east_42"));
obj->type();              // RoadObjectType::kBarrier
obj->subtype();           // "guardRail"
obj->name();              // "East guardrail section 42"
obj->position().inertial_position();   // where it is
obj->bounding_box();      // maliput::math::BoundingBox with position + dimensions
obj->related_lanes();     // which lanes it's next to
obj->properties();        // {"material": "steel"}

// Objects with precise geometry
if (obj->num_outlines() > 0) {
  const auto* outline = obj->outline(0);
  for (const auto& corner : outline->corners()) {
    // corner.position(), corner.height()
  }
}
```
