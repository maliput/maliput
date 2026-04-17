# TrafficSign API

## Overview

`TrafficSign` models a static, passive traffic sign — a physical signaling device placed along or above the road to convey regulatory, warning, or informational messages to road users. Examples include stop signs, yield signs, speed limit signs, no-entry signs, and pedestrian crossing signs.

**Namespace:** `maliput::api::rules`
**Headers:** `maliput/api/rules/traffic_sign.h`, `maliput/api/rules/traffic_sign_book.h`

## Key Design Principles

### Static and Passive

Unlike a `TrafficLight`, a traffic sign has **no dynamic state**. It doesn't change at runtime — it simply exists at a position with an orientation and a type. This fundamental difference justifies keeping `TrafficSign` and `TrafficLight` as separate classes despite their shared physical properties (position, orientation, bounding box).

### No Knowledge of Rules

Following the same pattern established by `TrafficLight`, a `TrafficSign` has **no reference to any Rule**. The linkage is always rule → sign:

- A `DiscreteValueRule` or `RangeValueRule` can reference a traffic sign's ID through its `RelatedUniqueIds` map.
- For example, a right-of-way rule at a stop-sign-controlled intersection would include `{"Traffic Sign Group": [stop_sign_id]}` in its related unique IDs.

This keeps physical objects decoupled from the rule system.

### Lane Association as a Spatial Fact

A traffic sign can declare which lanes it is physically relevant to via `related_lanes()`. This captures the geometric fact that a stop sign faces certain approach lanes — **not** a rule relationship. The precise longitudinal range where the sign's rule applies is determined by the rule's zone (`LaneSRange`), not by the sign itself.

## TrafficSign

| Property | Type | Description |
|----------|------|-------------|
| `id` | `TrafficSign::Id` | Unique identifier within the `TrafficSignBook` |
| `type` | `TrafficSignType` | Kind of sign (e.g., `kStop`, `kYield`, `kSpeedLimit`) |
| `position_road_network` | `InertialPosition` | Position in the road network's Inertial frame (center of sign face) |
| `orientation_road_network` | `Rotation` | Orientation in the Inertial frame (+X = facing vehicles, +Z = up) |
| `message` | `optional<string>` | Optional text on the sign (e.g., `"60"` on a speed limit sign) |
| `related_lanes` | `vector<LaneId>` | Lanes this sign is physically relevant to |
| `bounding_box` | `maliput::math::BoundingBox` | Bounding box in the sign's local frame |

### Sign Types

The `type` field is a `TrafficSignType` enum. It follows the [MUTCD](https://mutcd.fhwa.dot.gov/htm/2009/part2/part2a.htm) classification as a reference. A `TrafficSignTypeMapper()` function provides mapping to string representations.

| Enum Value | String | Description |
|------------|--------|-------------|
| `kStop` | `"Stop"` | Stop sign |
| `kYield` | `"Yield"` | Yield / give-way sign |
| `kSpeedLimit` | `"SpeedLimit"` | Speed limit sign (use `message` for the limit value) |
| `kNoEntry` | `"NoEntry"` | No entry / do not enter |
| `kOneWay` | `"OneWay"` | One-way traffic |
| `kPedestrianCrossing` | `"PedestrianCrossing"` | Pedestrian crossing warning |
| `kNoLeftTurn` | `"NoLeftTurn"` | No left turn |
| `kNoRightTurn` | `"NoRightTurn"` | No right turn |
| `kNoUTurn` | `"NoUTurn"` | No U-turn |
| `kSchoolZone` | `"SchoolZone"` | School zone |
| `kConstruction` | `"Construction"` | Construction / work zone |
| `kRailroadCrossing` | `"RailroadCrossing"` | Railroad crossing |
| `kUnknown` | `"Unknown"` | Unknown or unmapped signal |

### BoundingBox

The bounding box uses `maliput::math::BoundingBox`, which represents an oriented bounding box defined by a position, dimensions (`box_size`), orientation (`RollPitchYaw`), and tolerance. All constructor parameters are required — there are no defaults. A typical sign face is approximately 30" × 30" (0.762m × 0.762m) with minimal depth (0.05m).

## TrafficSignBook

`TrafficSignBook` is the abstract registry interface for traffic signs in a road network.

| Method | Returns | Description |
|--------|---------|-------------|
| `TrafficSigns()` | `vector<const TrafficSign*>` | All signs in the book |
| `GetTrafficSign(id)` | `const TrafficSign*` | A specific sign by ID, or `nullptr` |
| `FindByLane(lane_id)` | `vector<const TrafficSign*>` | All signs relevant to the given lane |
| `FindByType(type)` | `vector<const TrafficSign*>` | All signs of a given `TrafficSignType` (e.g., all stop signs) |

Backend implementations populate the book from their data sources.

## Relationship to TrafficLight

Both `TrafficSign` and `TrafficLight` are physical traffic control devices that live under `maliput::api::rules`. They are kept separate because:

| | TrafficSign | TrafficLight |
|-|-------------|--------------|
| **State** | Static (no runtime state) | Dynamic (bulb states change over time) |
| **Complexity** | Flat (single object) | Hierarchical (TrafficLight → BulbGroup → Bulb) |
| **Phase system** | Not involved | Integrated via `BulbStates` in `Phase` |
| **Internal structure** | Type + optional message | Bulbs with colors, types, arrow orientations |

## Example

```cpp
// Access signs from the road network
const auto* book = road_network->traffic_sign_book();

// Get all stop signs
auto stop_signs = book->FindByType(TrafficSignType::kStop);

// Get all signs relevant to a specific lane
auto signs_for_lane = book->FindByLane(LaneId("lane_42"));

// Inspect a specific sign
const auto* sign = book->GetTrafficSign(TrafficSign::Id("stop_sign_north"));
sign->type();                // TrafficSignType::kStop
sign->position_road_network(); // where it is
sign->related_lanes();       // which lanes it faces
sign->message();             // std::nullopt for a stop sign
```
