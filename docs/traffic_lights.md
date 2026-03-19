# TrafficLight API

## Overview

`TrafficLight` models a physical traffic light — a signaling device typically located at road intersections. It contains one or more groups of light bulbs with varying colors and shapes whose lighting patterns convey right-of-way information to agents navigating the intersection (vehicles, bicyclists, pedestrians, etc.).

**Namespace:** `maliput::api::rules`
**Headers:** `maliput/api/rules/traffic_lights.h`, `maliput/api/rules/traffic_light_book.h`

## Key Design Principles

### Stateful and Dynamic

Unlike traffic signs, a traffic light is a **dynamic** device. Its bulbs change state over time (`kOn`, `kOff`, `kBlinking`). The current state of a traffic light's bulbs is managed externally through the phase system:

```
PhaseProvider → Phase → BulbStates (map of UniqueBulbId → BulbState)
```

### No Knowledge of Rules

A `TrafficLight` is a purely physical entity. It knows about its position, orientation, and bulbs — but it has **no reference to any Rule**. The coupling goes in the opposite direction:

- A `DiscreteValueRule`'s `RelatedUniqueIds` can reference a `UniqueBulbId` or `UniqueBulbGroupId`.
- A `Phase`'s `BulbStates` maps `UniqueBulbId → BulbState`.

This decoupling means traffic lights can exist and be rendered without any rule infrastructure, and rules can reference lights without circular dependencies.

## Class Hierarchy

```
TrafficLight
 └── BulbGroup (one or more)
      └── Bulb (one or more)
```

### TrafficLight

Top-level object representing the entire physical traffic light assembly.

| Property | Type | Description |
|----------|------|-------------|
| `id` | `TrafficLight::Id` | Unique identifier within the `TrafficLightBook` |
| `position_road_network` | `InertialPosition` | Position in the road network's Inertial frame |
| `orientation_road_network` | `Rotation` | Orientation in the road network's Inertial frame (+Z = up) |
| `bulb_groups` | `vector<BulbGroup>` | The bulb groups that compose this traffic light |

### BulbGroup

A group of bulbs that share approximately the same orientation (e.g., the three-light vertical stack facing a particular approach lane).

| Property | Type | Description |
|----------|------|-------------|
| `id` | `BulbGroup::Id` | Unique within its parent `TrafficLight` |
| `position_traffic_light` | `InertialPosition` | Offset relative to the parent traffic light's frame |
| `orientation_traffic_light` | `Rotation` | Rotational offset relative to the parent traffic light's frame |
| `bulbs` | `vector<Bulb>` | The individual bulbs in this group |

### Bulb

A single light bulb with a color, type, and set of possible states.

| Property | Type | Description |
|----------|------|-------------|
| `id` | `Bulb::Id` | Unique within its parent `BulbGroup` |
| `position_bulb_group` | `InertialPosition` | Offset relative to the parent bulb group's frame |
| `orientation_bulb_group` | `Rotation` | Rotational offset relative to the parent bulb group's frame |
| `color` | `BulbColor` | `kRed`, `kYellow`, or `kGreen` |
| `type` | `BulbType` | `kRound`, `kArrow`, `kArrowLeft`, `kArrowRight`, `kArrowUp`, `kArrowUpperLeft`, `kArrowUpperRight`, `kUTurnLeft`, `kUTurnRight`, `kWalk`, or `kDontWalk` |
| `arrow_orientation_rad` | `optional<double>` | Arrow direction (only for `kArrow` type) |
| `states` | `vector<BulbState>` | Possible states: `kOff`, `kOn`, `kBlinking` |
| `bounding_box` | `BoundingBox` | Physical extents of the bulb |

## Unique Identifiers

Since bulbs and bulb groups are nested within traffic lights, two composite unique identifier classes are provided for global addressing:

- **`UniqueBulbId`** — `(TrafficLight::Id, BulbGroup::Id, Bulb::Id)` — uniquely identifies a single bulb across the entire road network.
- **`UniqueBulbGroupId`** — `(TrafficLight::Id, BulbGroup::Id)` — uniquely identifies a bulb group across the entire road network.

These are used by the phase system and rules to reference specific bulbs without ambiguity.

## TrafficLightBook

`TrafficLightBook` is the abstract registry interface that provides access to the traffic lights in a road network.

| Method | Description |
|--------|-------------|
| `TrafficLights()` | Returns all traffic lights |
| `GetTrafficLight(id)` | Returns a specific traffic light by ID, or `nullptr` |

Backend implementations (e.g., `maliput_malidrive`) populate the book from their data sources.

## Integration with the Phase System

Traffic lights are dynamic, and their state evolution is managed through phases:

```
PhaseRingBook → PhaseRing → Phase → BulbStates
                                   → DiscreteValueRuleStates
```

A `Phase` contains an optional `BulbStates` map (`UniqueBulbId → BulbState`) describing what each bulb should display during that phase. The `PhaseProvider` determines the currently active phase, thereby determining the current state of all traffic lights at an intersection.

## Example

```cpp
// Access a traffic light from the road network
const auto* book = road_network->traffic_light_book();
const auto* light = book->GetTrafficLight(TrafficLight::Id("intersection_1_north"));

// Inspect its structure
for (const auto* group : light->bulb_groups()) {
  for (const auto* bulb : group->bulbs()) {
    // bulb->color(), bulb->type(), bulb->states(), ...
  }
}

// Current state comes from the phase system, not the traffic light itself
const auto* phase_provider = road_network->phase_provider();
```
