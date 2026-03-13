# Road Furniture API — Planning Document

**Branch:** `slopez/road-furniture-api`  
**Date:** 2026-03-12  
**Status:** Draft — Q1–Q13 resolved; no open questions

---

## 1. Motivation

Maliput currently provides two complementary abstractions for road-associated entities:

- **`api/rules/`** — *semantic/behavioral* rules (speed limits as constraints, direction usage, right-of-way). These tell callers *what applies here*, but do not expose the physical sign or its position as a first-class object.
- **`api/objects/`** — *geometric* road objects (`RoadObject`, `RoadObjectBook`). These represent physical furniture items with position, orientation, bounding box, and outline, but without typed signal semantics.
- **`api/furniture/`** — currently empty stub files (`road_furniture.h`, `road_furniture_book.h`). This is the intended home for the new API.

The gap: there is no maliput API to ask *"What physical traffic signs exist along this route?"*, *"Which lanes does this stop sign apply to?"*, or *"Give me all speed limit signs whose posted value is ≤ 60 km/h"*.  Road Furniture fills this gap by lifting XODR (and future backend) signals into a typed, backend-agnostic maliput API.

---

## 2. Existing Context

### 2.1 `LaneSRoute` / `LaneSRange`
`maliput::api::LaneSRoute` is a sequence of `LaneSRange` objects, each identifying a lane by `LaneId` and a longitudinal `[s_min, s_max]` range. It is already used by routing and the rules API.

### 2.2 XODR Signal model (maliput_malidrive)
The malidrive XODR parser already fully parses `<signal>` elements into `xodr::signal::Signal` structs with:
- Position: `s`, `t`, `z_offset` (road-frame coordinates)
- Identification: `id`, `name`, `type`, `subtype`, `country`, `country_revision`
- Geometry: `length`, `width`, `height`, `h_offset`, `pitch`, `roll`
- Semantics: `dynamic`, `orientation` (`+`/`-`/`none`), optional `value` + `unit`
- Applicability: `validities` (XODR `<validity fromLane="…" toLane="…"/>` elements)

### 2.3 `RoadObject` API
`maliput::api::objects::RoadObject` already models static geometric furniture with position (`RoadObjectPosition`), orientation (`Rotation`), `BoundingBox`, `Outline`s, and a `properties` map. It uses the NVI pattern with a `RoadObjectBook` interface.

---

## 3. Resolved Questions

**[Q1] — Terminology: `RoadFurniture` vs. `TrafficSign` vs. `Signal`** ✅  
The top-level concept is `RoadFurniture`. All items (signs and physical objects) are represented directly as `RoadFurniture` instances — no subclasses. The `furniture_type()` field discriminates between item categories. Subclassing may be revisited in a later phase.

**[Q2] — Relationship to the existing `rules/` API** ✅  
`RoadFurniture` can be linked to rules. The API exposes:
```cpp
std::vector<rules::Rule::Id> RoadFurniture::GetRules() const;
```
Returning `Rule::Id`s (not pointers) keeps ownership clean — callers look up the actual rule objects in `RoadRulebook`. See follow-up [Q9].

**[Q3] — Signal type taxonomy** ✅  
Option C: a typed enum per category plus a `properties` map as an escape hatch for backend-specific attributes.

**Updated (Q10/Q12):** Sign and object types are unified into a single `RoadFurnitureType` enum; the former separate `RoadSignType` is superseded.
```cpp
enum class RoadFurnitureType { kUnknown, kSpeedLimitSign, kStopSign, kYieldSign, ..., kBarrier, ... };
```

**[Q4] — "Lanes affected by a signal"** ✅  
Return a flat `std::vector<LaneId>`. Evolution to `LaneSRange`-based results is deferred to a follow-up.

**[Q5] — "Signals in a LaneSRoute": positional or applicability-based** ✅  
Applicability-based: `GetByLaneSRoute()` returns all signs/objects that *apply to* any lane in the route.

**[Q6] — Speed-limit sign: typed accessor** ✅  
`SpeedLimitSign` is a concrete subclass of `RoadSign` with typed accessors (`max_speed()`, `min_speed()`, `unit()`).

**[Q7] — Dynamic signals** ✅  
Dynamic signs are in scope. Runtime state tracking is delegated entirely to the rules API via `GetRules()` and `DiscreteValueRuleStateProvider`. The `is_dynamic()` flag was removed from the furniture API (see resolved Q11).

**[Q8] — Integration with `RoadNetwork`** ✅  
`RoadFurnitureBook` is added as a nullable optional member of `RoadNetwork`, consistent with `TrafficLightBook`. Backends that do not implement it return `nullptr`.

---

## 4. Follow-up Questions

**[Q9] — `RoadObject` naming conflict** ✅  
`maliput::api::objects::RoadObject` already existed as an unofficial/inspiration class. The naming conflict is resolved by deprecating `api/objects/RoadObject` and making `api/furniture/RoadObject` its official replacement (option c). The complete hierarchy — `RoadFurniture`, `RoadSign`, `SpeedLimitSign`, `RoadObject` — lives in `api/furniture/`.

**[Q10] — `RoadFurnitureBook` type-query split** ✅  
All furniture types are united in a single `RoadFurnitureType` enum (option a). The book exposes a single `GetByType(RoadFurnitureType)` returning `std::vector<const RoadFurniture*>`. The bulk typed accessors `RoadSigns()` and `RoadObjects()` are retained for convenient pre-cast retrieval. This design may evolve later.

**[Q11] — Dynamic sign state representation** ✅  
`is_dynamic()` is **removed** from the `RoadFurniture` API. Dynamic sign behavior is fully delegated to the rules API. Callers correlate dynamic signs to their runtime state via `GetRules()` and `DiscreteValueRuleStateProvider`.

**[Q12] — `RoadObject` (furniture) type enum** ✅  
A single unified `RoadFurnitureType` enum is defined in `api/furniture/`, combining all former `RoadObjectType` categories with all former `RoadSignType` categories plus a shared `kUnknown` value. Both `api/objects::RoadObjectType` and the prior `RoadSignType` are superseded.

---

**[Q13] — Deprecation scope for `api/objects/`** ✅  
All three classes — `api/objects::RoadObject`, `api/objects::RoadObjectBook`, and `api/objects::RoadObjectType` — are moved into `api/furniture/` in Phase 1. `api/objects/` is left with deprecated forwarding headers that alias the new types for backward compatibility, and will be removed in a subsequent release. `RoadObjectBook` is superseded by `RoadFurnitureBook`; `RoadObjectType` is absorbed into `RoadFurnitureType`.

---

## 5. Proposed API Design
*(Updated to reflect resolved Q1–Q13)*

### 5.1 Class Hierarchy

```
RoadFurniture   (abstract — id, name, furniture_type, position, orientation, road_orientation,
                             affected_lane_ids, GetRules, properties)
                 └── TODO: bounding_box(), outlines() to be added once geometric helpers are
                           migrated from api/objects/ into api/furniture/
```

The former `RoadSign`, `SpeedLimitSign`, and `RoadObject` subclasses are removed. All furniture
items are represented directly as `RoadFurniture`. The `furniture_type()` accessor provides the
high-level category; type-specific typed accessors (e.g., `max_speed()`) are deferred to a
follow-up once the subclass design is revisited.

### 5.2 `maliput/api/furniture/road_furniture.h`

```cpp
#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/rules/rule.h"
#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace furniture {

/// Directional validity of a road furniture item relative to the road
/// reference line's s-direction.
enum class RoadFurnitureOrientation {
  kBoth,      ///< Applies to both directions of travel.
  kPositive,  ///< Applies only in the positive s-direction.
  kNegative,  ///< Applies only in the negative s-direction.
};

/// Unified type taxonomy for all road furniture items (signs and objects).
enum class RoadFurnitureType {
  kUnknown = 0,      ///< Unknown or unclassified item.
  // --- Sign types ---
  kSpeedLimitSign,   ///< Speed limit (maximum or advisory).
  kStopSign,         ///< Mandatory stop.
  kYieldSign,        ///< Yield / give-way.
  kNoEntrySign,      ///< No entry in this direction.
  kWarningSign,      ///< Generic warning.
  kInformationSign,  ///< Informational or directional sign.
  // --- Object types ---
  kBarrier,
  kBuilding,
  kCrosswalk,
  kGantry,
  kObstacle,
  kParkingSpace,
  kPole,
  kRoadMark,
  kRoadSurface,
  kTrafficIsland,
  kTree,
  kVegetation,
};

/// Abstract base class for all static road furniture.
///
/// All items (signs and physical objects) are represented directly as
/// RoadFurniture — no subclasses. Use furniture_type() to discriminate
/// between categories.
class RoadFurniture {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadFurniture)
  using Id = TypeSpecificIdentifier<class RoadFurniture>;

  virtual ~RoadFurniture() = default;

  /// Returns the unique identifier.
  const Id& id() const { return DoId(); }
  std::optional<std::string> name() const { return DoName(); }
  RoadFurnitureType furniture_type() const { return DoFurnitureType(); }
  const InertialPosition& position() const { return DoPosition(); }
  const Rotation& orientation() const { return DoOrientation(); }
  RoadFurnitureOrientation road_orientation() const { return DoRoadOrientation(); }
  std::vector<LaneId> affected_lane_ids() const { return DoAffectedLaneIds(); }
  std::vector<rules::Rule::Id> GetRules() const { return DoGetRules(); }
  const std::unordered_map<std::string, std::string>& properties() const { return DoProperties(); }

  // TODO(#road-furniture): Add bounding_box() and outlines() NVI methods once
  // the geometric helper types are migrated from api/objects/ into api/furniture/.

 protected:
  RoadFurniture() = default;

 private:
  virtual const Id& DoId() const = 0;
  virtual std::optional<std::string> DoName() const = 0;
  virtual RoadFurnitureType DoFurnitureType() const = 0;
  virtual const InertialPosition& DoPosition() const = 0;
  virtual const Rotation& DoOrientation() const = 0;
  virtual RoadFurnitureOrientation DoRoadOrientation() const = 0;
  virtual std::vector<LaneId> DoAffectedLaneIds() const = 0;
  virtual std::vector<rules::Rule::Id> DoGetRules() const = 0;
  virtual const std::unordered_map<std::string, std::string>& DoProperties() const = 0;
};

/// A speed limit sign.
///
/// Provides typed accessors for the posted speed range and unit.
/// Dynamic speed limit sign behavior (if applicable) is tracked at runtime
/// via the linked rule accessible through GetRules().
class SpeedLimitSign : public RoadFurniture {
 public:
  /// Returns the maximum posted speed.
  double max_speed() const { return DoMaxSpeed(); }

  /// Returns the minimum posted speed, if specified.
  std::optional<double> min_speed() const { return DoMinSpeed(); }

  /// Returns the unit string (e.g., "km/h", "mph").
  std::string unit() const { return DoUnit(); }

 protected:
  SpeedLimitSign() = default;

 private:
  virtual double DoMaxSpeed() const = 0;
  virtual std::optional<double> DoMinSpeed() const = 0;
  virtual std::string DoUnit() const = 0;
};

}  // namespace furniture
}  // namespace api
}  // namespace maliput
```

### 5.3 `maliput/api/furniture/road_furniture_book.h`

```cpp
#pragma once

#include <vector>

#include "maliput/api/lane_data.h"
#include "maliput/api/regions.h"
#include "maliput/api/furniture/road_furniture.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace api {
namespace furniture {

/// Abstract interface for querying RoadFurniture in a road network.
///
/// Follows the NVI pattern consistent with the rest of the maliput API
/// (cf. TrafficLightBook, RoadRulebook).
class RoadFurnitureBook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadFurnitureBook)
  virtual ~RoadFurnitureBook() = default;

  // -- Bulk accessors --------------------------------------------------------

  /// Returns all RoadFurniture items.
  std::vector<const RoadFurniture*> RoadFurnitures() const { return DoRoadFurnitures(); }

  // -- Lookup by ID ----------------------------------------------------------

  /// Returns the RoadFurniture with the given ID, or std::nullopt if not found.
  std::optional<const RoadFurniture*> GetRoadFurniture(const RoadFurniture::Id& id) const {
    return DoGetRoadFurniture(id);
  }

  // -- Lookup by type --------------------------------------------------------

  /// Returns all RoadFurniture items of the specified type.
  std::vector<const RoadFurniture*> GetByType(RoadFurnitureType type) const {
    return DoGetByType(type);
  }

  // -- Lookup by road location -----------------------------------------------

  /// Returns all RoadFurniture items that apply to any lane in @p route.
  std::vector<const RoadFurniture*> GetByLaneSRoute(const LaneSRoute& route) const {
    return DoGetByLaneSRoute(route);
  }

  /// Returns all RoadFurniture items that apply to the specified lane.
  std::vector<const RoadFurniture*> GetByLane(const LaneId& lane_id) const {
    return DoGetByLane(lane_id);
  }

  /// Returns the lanes affected by the furniture item with the given ID.
  std::vector<const Lane*> GetAffectedLanes(const RoadFurniture::Id& id) const {
    return DoGetAffectedLanes(id);
  }

 protected:
  RoadFurnitureBook() = default;

 private:
  virtual std::vector<const RoadFurniture*> DoRoadFurnitures() const = 0;
  virtual std::optional<const RoadFurniture*> DoGetRoadFurniture(const RoadFurniture::Id& id) const = 0;
  virtual std::vector<const RoadFurniture*> DoGetByType(RoadFurnitureType type) const = 0;
  virtual std::vector<const RoadFurniture*> DoGetByLaneSRoute(const LaneSRoute& route) const = 0;
  virtual std::vector<const RoadFurniture*> DoGetByLane(const LaneId& lane_id) const = 0;
  virtual std::vector<const Lane*> DoGetAffectedLanes(const RoadFurniture::Id& id) const = 0;
};

}  // namespace furniture
}  // namespace api
}  // namespace maliput
```

### 5.4 `RoadNetwork` integration

```cpp
// New constructor parameter (added at the end, optional / defaulted to nullptr
// to preserve backward compatibility with existing backends):
RoadNetwork(..., std::unique_ptr<const furniture::RoadFurnitureBook> road_furniture_book = nullptr);

// New accessor:
const furniture::RoadFurnitureBook* road_furniture_book() const {
  return road_furniture_book_.get();  // nullptr if backend does not provide it
}
```

---

## 6. API Design Decisions

| # | Decision | Resolution |
|---|----------|-----------|
| D1 | Top-level concept name | Flat `RoadFurniture` base class — no subclasses; `furniture_type()` discriminates categories |
| D2 | Type taxonomy | Unified `RoadFurnitureType` enum (sign categories with `Sign` suffix + object categories) + `properties` map for backend-specific details |
| D3 | Rules API coupling | Linked via `GetRules()` returning `std::vector<rules::Rule::Id>` |
| D4 | Affected lanes return type | `std::vector<const Lane*>` — evolution to `LaneSRange` is future work |
| D5 | LaneSRoute query semantics | Applicability-based (what applies to a route, not what's physically near it) |
| D6 | Dynamic signs | No `is_dynamic()` flag; dynamic behavior fully delegated to rules API via `GetRules()` |
| D7 | `RoadNetwork` integration | Nullable optional member, consistent with `TrafficLightBook` |
| D8 | `GetByType` | Single `GetByType(RoadFurnitureType)` returning `const RoadFurniture*`; typed bulk accessors (`RoadSigns()`/`RoadObjects()`) removed with the subclass hierarchy |
| D9 | `api/objects/` migration | All of `api/objects/` (`RoadObject`, `RoadObjectBook`, `RoadObjectType`) is moved into `api/furniture/`; `api/objects/` headers become deprecated forwarders removed in a later release |

---

## 7. Implementation Plan
*(Sequenced — each phase builds on the previous)*

### Phase 1 — maliput Core: Abstract API

**Location:** `maliput/include/maliput/api/furniture/`

Tasks:
1. Define `RoadFurnitureOrientation` and unified `RoadFurnitureType` enum in `road_furniture.h`.
2. Define `RoadFurniture` abstract base class (NVI) in `road_furniture.h`.
3. Define `RoadFurnitureBook` abstract interface (NVI) in `road_furniture_book.h`.
4. Add optional `road_furniture_book_` member and `road_furniture_book()` accessor to `RoadNetwork`.
5. Update `RoadNetwork` constructor(s) to accept the new optional book.
6. Migrate `api/objects/` into `api/furniture/`:
   - `api/objects::RoadObject` → superseded by `api/furniture::RoadFurniture` (no `RoadObject` subclass for now).
   - `api/objects::RoadObjectBook` → superseded by `api/furniture::RoadFurnitureBook`.
   - `api/objects::RoadObjectType` → absorbed into `api/furniture::RoadFurnitureType`.
   - Leave deprecated forwarding headers in `api/objects/` that include the new `api/furniture/` headers and emit `[[deprecated]]` warnings. These forwarding headers will be removed in a follow-up release.
7. Add CMake targets and install rules in `maliput/CMakeLists.txt`.
8. Add unit tests with mock implementations in `maliput/test/`.

### Phase 2 — maliput_malidrive: Backend Implementation

**Location:** `maliput_malidrive/src/maliput_malidrive/`

Tasks:
1. Create `MalidriveSpeedLimitSign` — concrete `SpeedLimitSign` subclass.
   - Populates `max_speed`, `min_speed`, `unit` from `xodr::signal::Signal::Value`.
2. Create `MalidriveRoadSign` — concrete `RoadSign` subclass for all other sign types.
   - Maps XODR `type`/`subtype`/`country` → `RoadFurnitureType` via a lookup table.
   - Translates XODR `orientation` (`+`/`-`/`none`) → `RoadFurnitureOrientation`.
   - Stores XODR-specific attributes in `properties`: `"country"`, `"xodr_type"`, `"xodr_subtype"`, `"country_revision"`.
   - Resolves `validities` (fromLane/toLane) to a `std::vector<LaneId>`.
   - Links to a `RangeValueRule::Id` or `DiscreteValueRule::Id` in `GetRules()` if a matching rule exists.
3. Create `MalidriveRoadObject` — concrete `RoadObject` subclass for XODR `<object>` elements.
4. Create `MalidriveRoadFurnitureBook` — concrete `RoadFurnitureBook` subclass.
   - Builds the collection during road network loading (after XODR parse).
   - Indexes items by `LaneId` for efficient `GetByLane` and `GetByLaneSRoute`.
5. Wire `MalidriveRoadFurnitureBook` into `MalidriveRoadNetworkBuilder`.
6. Pass ownership to `RoadNetwork` constructor.
7. Add integration tests using existing XODR resources with known signals.

### Phase 3 — Validation and Documentation

1. Optionally extend `road_network_validator` to check furniture book consistency.
2. Write Doxygen `@par` examples for key query patterns.

---

## 8. Key Constraints and Design Principles

- **NVI pattern** — all public methods are non-virtual and delegate to private pure-virtual `Do*` methods, consistent with `TrafficLightBook`, `RoadRulebook`.
- **Base implementations** — if implementations are general enough, a base class can be added into `maliput/base` directory.
- **Backend-agnostic types** — no XODR-specific types leak into the maliput core API.
- **Nullability** — `RoadNetwork::road_furniture_book()` returns `nullptr` for backends that do not implement it, preserving backward compatibility.
- **No breaking changes** — `RoadFurnitureBook` is added as an optional parameter to `RoadNetwork`; existing backends are unaffected.
- **`properties` as escape hatch** — backend-specific data (country, XODR type strings) is accessible without polluting the typed API.
- **Rules decoupling** — `GetRules()` returns only `Rule::Id`s; the furniture API does not own or cache rule objects.

---

## 9. Out of Scope (v1)

- Runtime furniture state changes (dynamic sign value tracking at the furniture API level — delegated to the rules API).
- Signals from non-malidrive backends (maliput_multilane, maliput_dragway) — those return empty books.
- Traffic lights (covered by `TrafficLightBook`).
- `LaneSRange`-based affected-lane queries (future evolution from the current `const Lane*`-list approach).
- Pedestrian/cyclist-specific or country-specific sign semantics beyond what `properties` provides.

