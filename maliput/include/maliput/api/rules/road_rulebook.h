#pragma once

#include <map>
#include <vector>

#include "maliput/api/regions.h"
#include "maliput/api/rules/direction_usage_rule.h"
#include "maliput/api/rules/discrete_value_rule.h"
#include "maliput/api/rules/range_value_rule.h"
#include "maliput/api/rules/right_of_way_rule.h"
#include "maliput/api/rules/speed_limit_rule.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/common/maliput_throw.h"

namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for querying "rules of the road".  This interface
/// provides access to static information about a road network (i.e.,
/// information determined prior to the beginning of a simulation).  Some
/// rule types may refer to additional dynamic information which will be
/// provided by other interfaces.  (For example, see RightOfWayRule.)
///
/// Concrete implementations of this interface shall be provided by
/// implementing the pure virtual methods declared in private scope.
class RoadRulebook {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(RoadRulebook);

  virtual ~RoadRulebook() = default;

  /// Results of a FindRules() query.  Results are organized by type; an
  /// empty map indicates no applicable rules of that type are known.
  struct QueryResults {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    std::map<RightOfWayRule::Id, RightOfWayRule> right_of_way;
    std::map<SpeedLimitRule::Id, SpeedLimitRule> speed_limit;
    std::map<DirectionUsageRule::Id, DirectionUsageRule> direction_usage;
#pragma GCC diagnostic pop
    std::map<DiscreteValueRule::Id, DiscreteValueRule> discrete_value_rules;
    std::map<RangeValueRule::Id, RangeValueRule> range_value_rules;
  };

  /// Returns a QueryResults structure which contains any rules which are
  /// applicable to the provided `ranges`.
  ///
  /// `tolerance` is the acceptable linear-tolerance in longitudinal
  /// s-coordinate in each range and must be non-negative.  A non-zero
  /// `tolerance` makes the query more permissive.  However, a non-zero
  /// `tolerance` does not permit matching across BranchPoints (past the
  /// s-bounds of a Lane).
  ///
  /// @throws maliput::common::assertion_error if `tolerance` is negative.
  QueryResults FindRules(const std::vector<LaneSRange>& ranges, double tolerance) const {
    MALIPUT_THROW_UNLESS(tolerance >= 0.);
    return DoFindRules(ranges, tolerance);
  }

  /// Returns all the rules in this RoadRulebook.
  QueryResults Rules() const { return DoRules(); }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  /// Returns the RightOfWayRule with the specified `id`.
  ///
  /// @throws std::out_of_range if `id` is unknown.
  MALIPUT_DEPRECATED("next release", "RightOfWayRule class will be deprecated.")
  RightOfWayRule GetRule(const RightOfWayRule::Id& id) const { return DoGetRule(id); }

  /// Returns the SpeedLimitRule with the specified `id`.
  ///
  /// @throws std::out_of_range if `id` is unknown.
  MALIPUT_DEPRECATED("next release", "SpeedLimitRule class will be deprecated.")
  SpeedLimitRule GetRule(const SpeedLimitRule::Id& id) const { return DoGetRule(id); }

  /// Returns the DirectionUsageRule with the specified `id`.
  ///
  /// @throws std::out_of_range if `id` is unknown.
  MALIPUT_DEPRECATED("next release", "DirectionUsageRule class will be deprecated.")
  DirectionUsageRule GetRule(const DirectionUsageRule::Id& id) const { return DoGetRule(id); }
#pragma GCC diagnostic pop

  /// Returns the DiscreteValueRule with the specified `id`.
  ///
  /// @throws std::out_of_range if `id` is unknown.
  DiscreteValueRule GetDiscreteValueRule(const Rule::Id& id) const { return DoGetDiscreteValueRule(id); }

  /// Returns the RangeValueRule with the specified `id`.
  ///
  /// @throws std::out_of_range if `id` is unknown.
  RangeValueRule GetRangeValueRule(const Rule::Id& id) const { return DoGetRangeValueRule(id); }

 protected:
  RoadRulebook() = default;

 private:
  // @name NVI implementations of the public methods.
  // These must satisfy the constraints/invariants of the
  // corresponding public methods.
  //@{
  virtual QueryResults DoFindRules(const std::vector<LaneSRange>& ranges, double tolerance) const = 0;
  virtual QueryResults DoRules() const = 0;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  virtual RightOfWayRule DoGetRule(const RightOfWayRule::Id& id) const = 0;
  virtual SpeedLimitRule DoGetRule(const SpeedLimitRule::Id& id) const = 0;
  virtual DirectionUsageRule DoGetRule(const DirectionUsageRule::Id& id) const = 0;
#pragma GCC diagnostic pop
  virtual DiscreteValueRule DoGetDiscreteValueRule(const Rule::Id& id) const = 0;
  virtual RangeValueRule DoGetRangeValueRule(const Rule::Id& id) const = 0;
  //@}
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
