#include "maliput/base/manual_rulebook.h"

#include <algorithm>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "drake/common/drake_optional.h"
#include "drake/common/hash.h"

#include "maliput/api/rules/rule.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {

using api::LaneId;
using api::LaneSRange;
using api::SRange;
using api::rules::DirectionUsageRule;
using api::rules::DiscreteValueRule;
using api::rules::RangeValueRule;
using api::rules::RightOfWayRule;
using api::rules::Rule;
using api::rules::SpeedLimitRule;

using QueryResults = api::rules::RoadRulebook::QueryResults;

namespace {

// TODO(maddog@tri.global)  When std::variant becomes available, use it like so:
// using IdVariant = std::variant<RightOfWayRule::Id,
//                                SpeedLimitRule::Id>;
struct IdVariant {
  drake::optional<RightOfWayRule::Id> r;
  drake::optional<SpeedLimitRule::Id> s;
  drake::optional<DirectionUsageRule::Id> d;
  drake::optional<Rule::Id> rule;

  IdVariant() = default;

  // NOLINTNEXTLINE(runtime/explicit)
  IdVariant(const RightOfWayRule::Id& r_in) : r(r_in) {}

  // NOLINTNEXTLINE(runtime/explicit)
  IdVariant(const SpeedLimitRule::Id& s_in) : s(s_in) {}

  // NOLINTNEXTLINE(runtime/explicit)
  IdVariant(const DirectionUsageRule::Id& d_in) : d(d_in) {}

  // NOLINTNEXTLINE(runtime/explicit)
  IdVariant(const Rule::Id& rule_in) : rule(rule_in) {}

  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const IdVariant& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.r);
    hash_append(hasher, item.s);
    hash_append(hasher, item.d);
    hash_append(hasher, item.rule);
  }
};

bool operator==(const IdVariant& lhs, const IdVariant& rhs) {
  return (lhs.r == rhs.r) && (lhs.s == rhs.s) && (lhs.d == rhs.d) && (lhs.rule == rhs.rule);
}

}  // namespace

class ManualRulebook::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() : index_(std::make_unique<RangeIndex>()) {}
  ~Impl() {}

  void RemoveAll() {
    right_of_ways_.clear();
    speed_limits_.clear();
    direction_usage_rules_.clear();
    discrete_value_rules_.clear();
    range_value_rules_.clear();
    index_->RemoveAll();
  }

  void AddRule(const api::rules::RightOfWayRule& rule) { AddAnyRule(rule, &right_of_ways_); }

  void RemoveRule(const api::rules::RightOfWayRule::Id& id) { RemoveAnyRule(id, &right_of_ways_); }

  void AddRule(const api::rules::SpeedLimitRule& rule) { AddAnyRule(rule, &speed_limits_); }

  void RemoveRule(const api::rules::SpeedLimitRule::Id& id) { RemoveAnyRule(id, &speed_limits_); }

  void AddRule(const api::rules::DirectionUsageRule& rule) { AddAnyRule(rule, &direction_usage_rules_); }

  void RemoveRule(const api::rules::DirectionUsageRule::Id& id) { RemoveAnyRule(id, &direction_usage_rules_); }

  void AddRule(const api::rules::DiscreteValueRule& rule) {
    MALIPUT_THROW_UNLESS(range_value_rules_.find(rule.id()) == range_value_rules_.end());

    MALIPUT_THROW_UNLESS(discrete_value_rules_.emplace(rule.id(), rule).second);
    index_->AddRule(rule);
  }

  void AddRule(const api::rules::RangeValueRule& rule) {
    MALIPUT_THROW_UNLESS(discrete_value_rules_.find(rule.id()) == discrete_value_rules_.end());

    MALIPUT_THROW_UNLESS(range_value_rules_.emplace(rule.id(), rule).second);
    index_->AddRule(rule);
  }

  void RemoveRule(const api::rules::Rule::Id& id) {
    if (discrete_value_rules_.find(id) != discrete_value_rules_.end()) {
      // Remove from index.
      index_->RemoveRule(discrete_value_rules_.at(id));
      // Remove from map.
      MALIPUT_THROW_UNLESS(discrete_value_rules_.erase(id) > 0);
    } else if (range_value_rules_.find(id) != range_value_rules_.end()) {
      // Remove from index.
      index_->RemoveRule(range_value_rules_.at(id));
      // Remove from map.
      MALIPUT_THROW_UNLESS(range_value_rules_.erase(id) > 0);
    } else {
      MALIPUT_THROW_MESSAGE("Unable to remove Rule: Rule::Id: " + id.string() + " cannot be found.");
    }
  }

  QueryResults DoFindRules(const std::vector<LaneSRange>& ranges, double tolerance) const {
    QueryResults result;
    for (const LaneSRange& range : ranges) {
      for (const IdVariant& id : index_->FindRules(range, tolerance)) {
        if (id.r) {
          result.right_of_way.emplace(*id.r, right_of_ways_.at(*id.r));
        } else if (id.s) {
          result.speed_limit.emplace(*id.s, speed_limits_.at(*id.s));
        } else if (id.d) {
          result.direction_usage.emplace(*id.d, direction_usage_rules_.at(*id.d));
        } else if (id.rule) {
          if (range_value_rules_.find(*id.rule) != range_value_rules_.end()) {
            result.range_value_rules.emplace(*id.rule, range_value_rules_.at(*id.rule));
          } else if (discrete_value_rules_.find(*id.rule) != discrete_value_rules_.end()) {
            result.discrete_value_rules.emplace(*id.rule, discrete_value_rules_.at(*id.rule));
          } else {
            throw std::out_of_range("IdVariant::rule:" + id.rule->string() + " could not be found.");
          }
        } else {
          std::stringstream s;
          s << "ManualRulebook: IdVariant is empty (LaneId: " << range.lane_id().string()
            << ", s0: " << range.s_range().s0() << ", s1: " << range.s_range().s1() << ")";
          throw std::domain_error(s.str());
        }
      }
    }
    return result;
  }

  QueryResults DoRules() const {
    QueryResults result;
    result.right_of_way.insert(right_of_ways_.begin(), right_of_ways_.end());
    result.speed_limit.insert(speed_limits_.begin(), speed_limits_.end());
    result.direction_usage.insert(direction_usage_rules_.begin(), direction_usage_rules_.end());
    result.discrete_value_rules.insert(discrete_value_rules_.begin(), discrete_value_rules_.end());
    result.range_value_rules.insert(range_value_rules_.begin(), range_value_rules_.end());
    return result;
  }

  RightOfWayRule DoGetRule(const RightOfWayRule::Id& id) const { return GetAnyRule(id, right_of_ways_); }

  SpeedLimitRule DoGetRule(const SpeedLimitRule::Id& id) const { return GetAnyRule(id, speed_limits_); }

  DirectionUsageRule DoGetRule(const DirectionUsageRule::Id& id) const {
    return GetAnyRule(id, direction_usage_rules_);
  }

  DiscreteValueRule DoGetDiscreteValueRule(const Rule::Id& id) const { return discrete_value_rules_.at(id); }

  RangeValueRule DoGetRangeValueRule(const Rule::Id& id) const { return range_value_rules_.at(id); }

 private:
  // An index from LaneSRange to collections of rule ID's of all types.
  // RangeIndex indexes rules (by ID) on the LaneSRanges which they affect,
  // facilitating the lookup of rules by LaneSRange.
  class RangeIndex {
   public:
    void RemoveAll() { map_.clear(); }

    void AddRule(const SpeedLimitRule& rule) { AddRange(rule.id(), rule.zone()); }

    void RemoveRule(const SpeedLimitRule& rule) { RemoveRanges(rule.id(), rule.zone().lane_id()); }

    void AddRule(const RightOfWayRule& rule) {
      for (const LaneSRange& range : rule.zone().ranges()) {
        AddRange(IdVariant(rule.id()), range);
      }
    }

    void RemoveRule(const RightOfWayRule& rule) {
      for (const LaneSRange& range : rule.zone().ranges()) {
        RemoveRanges(IdVariant(rule.id()), range.lane_id());
      }
    }

    void AddRule(const DirectionUsageRule& rule) { AddRange(rule.id(), rule.zone()); }

    void RemoveRule(const DirectionUsageRule& rule) { RemoveRanges(rule.id(), rule.zone().lane_id()); }

    void AddRule(const DiscreteValueRule& rule) {
      for (const LaneSRange& range : rule.zone().ranges()) {
        AddRange(rule.id(), range);
      }
    }

    void RemoveRule(const DiscreteValueRule& rule) {
      for (const LaneSRange& range : rule.zone().ranges()) {
        RemoveRanges(rule.id(), range.lane_id());
      }
    }

    void AddRule(const RangeValueRule& rule) {
      for (const LaneSRange& range : rule.zone().ranges()) {
        AddRange(rule.id(), range);
      }
    }

    void RemoveRule(const RangeValueRule& rule) {
      for (const LaneSRange& range : rule.zone().ranges()) {
        RemoveRanges(rule.id(), range.lane_id());
      }
    }

    std::vector<IdVariant> FindRules(const LaneSRange& range, double tolerance) {
      std::vector<IdVariant> result;
      auto it = map_.find(range.lane_id());
      if (it != map_.end()) {
        for (const auto& id_and_range : it->second) {
          if (id_and_range.second.Intersects(range.s_range(), tolerance)) {
            result.emplace_back(id_and_range.first);
          }
        }
      }
      return result;
    }

   private:
    // Add a single (ID, LaneSRange) association.
    void AddRange(const IdVariant& id, const LaneSRange& range) { map_[range.lane_id()].emplace(id, range.s_range()); }

    // Removes all associations involving `id` and `lane_id`.
    void RemoveRanges(const IdVariant& id, const LaneId& lane_id) {
      map_.at(lane_id).erase(id);
      if (map_[lane_id].empty()) {
        map_.erase(lane_id);
      }
    }

    // TODO(maddog@tri.global)  Perhaps this class would benefit from something
    //                          like boost::interval_map, though if there are
    //                          not many rules attached to each individual
    //                          lane_id, this is probably good enough.
    std::unordered_map<LaneId, std::unordered_multimap<IdVariant, SRange, drake::DefaultHash>> map_;
  };

  // ID->Rule indices for each rule type.
  template <class T>
  using IdIndex = std::unordered_map<typename T::Id, T>;

  template <class T>
  void AddAnyRule(const T& rule, IdIndex<T>* map) {
    // Add to map.
    auto map_result = map->emplace(rule.id(), rule);
    // Throw if the id was already present.
    MALIPUT_THROW_UNLESS(map_result.second);
    // Add to index.
    index_->AddRule(rule);
  }

  template <class T>
  T GetAnyRule(const typename T::Id& id, const IdIndex<T>& map) const {
    return map.at(id);
  }

  template <class T>
  void RemoveAnyRule(const typename T::Id& id, IdIndex<T>* map) {
    MALIPUT_THROW_UNLESS(map->count(id) == 1);
    // Remove from index.
    index_->RemoveRule(map->at(id));
    // Remove from map.
    auto map_result = map->erase(id);
    MALIPUT_THROW_UNLESS(map_result > 0);
  }

  std::unique_ptr<RangeIndex> index_;
  IdIndex<api::rules::RightOfWayRule> right_of_ways_;
  IdIndex<api::rules::SpeedLimitRule> speed_limits_;
  IdIndex<api::rules::DirectionUsageRule> direction_usage_rules_;
  std::unordered_map<Rule::Id, DiscreteValueRule> discrete_value_rules_;
  std::unordered_map<Rule::Id, RangeValueRule> range_value_rules_;
};  // namespace maliput

ManualRulebook::ManualRulebook() : impl_(std::make_unique<Impl>()) {}

ManualRulebook::~ManualRulebook() = default;

void ManualRulebook::RemoveAll() { impl_->RemoveAll(); }

void ManualRulebook::AddRule(const api::rules::RightOfWayRule& rule) { impl_->AddRule(rule); }

void ManualRulebook::RemoveRule(const api::rules::RightOfWayRule::Id& id) { impl_->RemoveRule(id); }

void ManualRulebook::AddRule(const api::rules::SpeedLimitRule& rule) { impl_->AddRule(rule); }

void ManualRulebook::RemoveRule(const api::rules::SpeedLimitRule::Id& id) { impl_->RemoveRule(id); }

void ManualRulebook::AddRule(const api::rules::DirectionUsageRule& rule) { impl_->AddRule(rule); }

void ManualRulebook::RemoveRule(const api::rules::DirectionUsageRule::Id& id) { impl_->RemoveRule(id); }

void ManualRulebook::AddRule(const api::rules::DiscreteValueRule& rule) { impl_->AddRule(rule); }

void ManualRulebook::AddRule(const api::rules::RangeValueRule& rule) { impl_->AddRule(rule); }

void ManualRulebook::RemoveRule(const api::rules::Rule::Id& id) { impl_->RemoveRule(id); }

QueryResults ManualRulebook::DoFindRules(const std::vector<LaneSRange>& ranges, double tolerance) const {
  return impl_->DoFindRules(ranges, tolerance);
}

QueryResults ManualRulebook::DoRules() const { return impl_->DoRules(); }

RightOfWayRule ManualRulebook::DoGetRule(const RightOfWayRule::Id& id) const { return impl_->DoGetRule(id); }

SpeedLimitRule ManualRulebook::DoGetRule(const SpeedLimitRule::Id& id) const { return impl_->DoGetRule(id); }

DirectionUsageRule ManualRulebook::DoGetRule(const DirectionUsageRule::Id& id) const { return impl_->DoGetRule(id); }

DiscreteValueRule ManualRulebook::DoGetDiscreteValueRule(const Rule::Id& id) const {
  return impl_->DoGetDiscreteValueRule(id);
}

RangeValueRule ManualRulebook::DoGetRangeValueRule(const Rule::Id& id) const { return impl_->DoGetRangeValueRule(id); }

}  // namespace maliput
