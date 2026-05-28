# RoadMarking API

## Overview

`RoadMarking` models a physical road marking — a painted or applied marking on the road surface that conveys regulatory, warning, or guidance information to road users. Examples include stop markings, crosswalks, directional arrows, speed limit markings, parking space delineations, and railroad crossings.

**Namespace:** `maliput::api::objects`
**Headers:** `maliput/api/objects/road_marking.h`, `maliput/api/objects/road_marking_book.h`

## Key Design Principles

### Purely Physical / Geometric

A `RoadMarking` is a spatial entity with a position, orientation, bounding box, and type. It represents something that physically exists on the road surface — something an agent might need to perceive, react to, or use for navigation.

### No Knowledge of Rules

Like `TrafficLight`, `TrafficSign`, and `RoadObject`, a `RoadMarking` has **no reference to any Rule**. If a road marking implies a rule (e.g., a stop line implies a stop rule), the rule references the marking through `Rule::State::related_unique_ids` — not the other way around.

### Static and Final

`RoadMarking` is a `final` class — it cannot be subclassed. Road markings are static; they do not change position or state at runtime.

### Coexists with RoadObject

Although a `RoadMarking` could essentialy be a `RoadObject`, these provide some semantic information through their type that distinguish them from `RoadObject`s.
The two APIs serve complementary purposes: `RoadObject` for generic physical objects, and `RoadMarking` for semantically rich road surface markings.

## Supporting Types

### RoadMarkingType (enum)

Type-safe enumeration of road marking categories:

| Enum Value | String | Description |
|------------|--------|-------------|
| `kStop` | `"Stop"` | Stop marking painted on the road |
| `kStopLine` | `"StopLine"` | Stop line across the lane |
| `kCrosswalk` | `"Crosswalk"` | Pedestrian crossing marking |
| `kParkingSpace` | `"ParkingSpace"` | Parking space delineation |
| `kEmergencyLane` | `"EmergencyLane"` | Emergency lane marking |
| `kSpeedLimit` | `"SpeedLimit"` | Speed limit marking (use `GetValue()` for the limit) |
| `kDoNotStop` | `"DoNotStop"` | Do-not-stop zone marking |
| `kRailRoad` | `"RailRoad"` | Railroad crossing marking |
| `kGiveWay` | `"GiveWay"` | Give way / yield marking |
| `kArrowTurnRight` | `"ArrowTurnRight"` | Right turn arrow |
| `kArrowTurnLeft` | `"ArrowTurnLeft"` | Left turn arrow |
| `kArrowForwardTurnRight` | `"ArrowForwardTurnRight"` | Forward + right turn arrow |
| `kArrowForwardTurnLeft` | `"ArrowForwardTurnLeft"` | Forward + left turn arrow |
| `kArrowForward` | `"ArrowForward"` | Forward arrow |
| `kArrowForwardTurnRightTurnLeft` | `"ArrowForwardTurnRightTurnLeft"` | Forward + right + left arrow |
| `kArrowTurnRightTurnLeft` | `"ArrowTurnRightTurnLeft"` | Right + left turn arrow |
| `kArrowUTurnRight` | `"ArrowUTurnRight"` | Right U-turn arrow |
| `kArrowUTurnLeft` | `"ArrowUTurnLeft"` | Left U-turn arrow |
| `kUnknown` | `"Unknown"` | Unknown / unclassified marking |

### RoadMarkingValueUnit (enum)

Units for numeric values associated with road markings (primarily speed limits):

| Enum Value | String | Description |
|------------|--------|-------------|
| `kMetersPerSecond` | `"m/s"` | Meters per second |
| `kKilometersPerHour` | `"km/h"` | Kilometers per hour |
| `kMilesPerHour` | `"mph"` | Miles per hour |

### RoadMarkingValue (struct)

Holds a numeric value and its associated unit:

| Field | Type | Description |
|-------|------|-------------|
| `value` | `double` | The numeric value |
| `unit` | `RoadMarkingValueUnit` | The unit of measurement |

### BoundingBox (`maliput::math::BoundingBox`)

`RoadMarking` reuses the existing `maliput::math::BoundingBox` class for coarse spatial representation. For road markings, this is typically a flat box on the road surface (minimal height).

### Outline / OutlineCorner

`RoadMarking` reuses the `Outline` and `OutlineCorner` classes from `maliput::api::objects`. When outlines are defined, they supersede the bounding box for precise geometry representation. This is useful for complex marking shapes like crosswalk zebra patterns or irregularly shaped markings.

### RoadObjectPosition

`RoadMarking` uses `RoadObjectPosition` from `maliput::api::objects`, providing:

- **Inertial position** (always available) — global (x, y, z) coordinates.
- **Lane-relative position** (optional) — `LaneId` + `LanePosition(s, r, h)` for markings semantically associated with a specific lane.

## RoadMarking

| Property | Type | Description |
|----------|------|-------------|
| `id` | `RoadMarking::Id` | Unique identifier within the `RoadMarkingBook` |
| `name` | `optional<string>` | Human-readable name for debugging/display |
| `type` | `RoadMarkingType` | Marking category (enum) |
| `position` | `RoadObjectPosition` | Inertial + optional lane-relative position |
| `orientation` | `Rotation` | Orientation in the inertial frame |
| `bounding_box` | `maliput::math::BoundingBox` | Bounding volume |
| `related_lanes` | `vector<LaneId>` | All lanes this marking is spatially relevant to |
| `outlines` | `vector<unique_ptr<Outline>>` | Detailed shape geometry (supersedes bounding box) |
| `GetValue()` | `optional<RoadMarkingValue>` | Numeric value (e.g., speed limit + unit) |

## RoadMarkingBook

`RoadMarkingBook` is the abstract registry interface for road markings. It provides the NVI pattern used throughout maliput.

| Method | Returns | Description |
|--------|---------|-------------|
| `RoadMarkings()` | `vector<const RoadMarking*>` | All markings in the book |
| `GetRoadMarking(id)` | `const RoadMarking*` | A specific marking by ID, or `nullptr` |
| `FindByType(type)` | `vector<const RoadMarking*>` | All markings of a given `RoadMarkingType` |
| `FindByLane(lane_id)` | `vector<const RoadMarking*>` | All markings relevant to the given lane |

Backend implementations populate the book from their data sources (e.g., OpenDRIVE signal elements with road marking subtypes).

## See Also

- [road_object.md](road_object.md) — `RoadObject` API for general physical objects.
- [traffic_sign.md](traffic_sign.md) — `TrafficSign` API for traffic signs.
- [regulatory_elements_and_objects.md](regulatory_elements_and_objects.md) — comparison of regulatory element and object APIs.

## Example

```cpp
#include "maliput/api/objects/road_marking.h"
#include "maliput/api/objects/road_marking_book.h"

using namespace maliput::api::objects;

// Access road markings
const auto* book = road_network->road_marking_book();

// Get all crosswalk markings
auto crosswalks = book->FindByType(RoadMarkingType::kCrosswalk);

// Get all markings relevant to a specific lane
auto markings_for_lane = book->FindByLane(LaneId("1_0_7"));

// Inspect a specific marking
const auto* marking = book->GetRoadMarking(RoadMarking::Id("speed_limit_60_lane3"));
marking->type();              // RoadMarkingType::kSpeedLimit
marking->name();              // "Speed Limit 60"
marking->position().inertial_position();   // where it is
marking->bounding_box();      // maliput::math::BoundingBox
marking->related_lanes();     // which lanes it spans

// Get the speed limit value
if (marking->GetValue().has_value()) {
  const auto& val = marking->GetValue().value();
  // val.value == 60.0, val.unit == RoadMarkingValueUnit::kKilometersPerHour
}

// Markings with precise geometry
if (marking->num_outlines() > 0) {
  const auto* outline = marking->outline(0);
  for (const auto& corner : outline->corners()) {
    // corner.position(), corner.height()
  }
}
```
