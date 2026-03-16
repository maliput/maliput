# Regulatory Elements and Objects

## Overview

Maliput provides three complementary APIs to model the physical entities in a road environment that regulate, signal, or influence traffic beyond the road surface itself:

```
                        Physical Entities
                              │
              ┌───────────────┼───────────────┐
              │               │               │
         RoadObject     TrafficSign     TrafficLight
      (api/objects/)    (api/rules/)    (api/rules/)
              │               │               │
         Environmental   Static traffic  Dynamic traffic
         features        control devices control devices
              │               │               │
         barriers,       stop signs,     traffic lights
         crosswalks,     speed limit     with bulbs and
         buildings,      signs, etc.     state changes
         trees, etc.
```

## Comparison

| | RoadObject | TrafficSign | TrafficLight |
|-|------------|-------------|--------------|
| **Namespace** | `maliput::api::objects` | `maliput::api::rules` | `maliput::api::rules` |
| **Nature** | Environmental | Regulatory signage | Active signaling |
| **State** | Static (position fixed, may have movable parts) | Static | Dynamic (bulb states) |
| **Knows about rules** | No | No | No |
| **Rules reference it via** | `Rule::State::related_unique_ids` | `Rule::State::related_unique_ids` | `Rule::State::related_unique_ids` / `BulbStates` |
| **Lane association** | `related_lanes()` + `RoadObjectPosition` | `related_lanes()` | — |
| **Extensibility** | `subtype` + `properties` + `Outline` | `message` field | Bulb hierarchy |
| **Spatial query** | `FindInRadius()` | `FindByLane()` | — |

## Design Principle: No Rule Knowledge

All three APIs share the same fundamental design principle: **physical objects have no knowledge of rules**. Rules reference objects — never the other way around.

- A `DiscreteValueRule` or `RangeValueRule` references objects through `Rule::State::related_unique_ids`.
- Traffic lights are additionally referenced through `Phase::BulbStates`.
- The convention for group names in `related_unique_ids` is:
  - `"Road Object Group"` — for `RoadObject` references.
  - `"Traffic Sign Group"` — for `TrafficSign` references.
  - `"Bulb Group"` — for `TrafficLight` bulb references.

This decoupling keeps the physical model clean and allows rules to be defined, modified, or loaded independently of the objects they govern.

## See Also

- [road_object.md](road_object.md) — `RoadObject` and `RoadObjectBook` API details.
- [traffic_sign.md](traffic_sign.md) — `TrafficSign` and `TrafficSignBook` API details.
- [traffic_lights.md](traffic_lights.md) — `TrafficLight`, `BulbGroup`, `Bulb`, and `TrafficLightBook` API details.
