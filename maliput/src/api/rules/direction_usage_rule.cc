#include "maliput/api/rules/direction_usage_rule.h"

namespace maliput {
namespace api {
namespace rules {

std::unordered_map<DirectionUsageRule::State::Type, const char*, drake::DefaultHash>
DirectionUsageRule::StateTypeMapper() {
  return {{DirectionUsageRule::State::Type::kWithS, "WithS"},
          {DirectionUsageRule::State::Type::kAgainstS, "AgainstS"},
          {DirectionUsageRule::State::Type::kBidirectional, "Bidirectional"},
          {DirectionUsageRule::State::Type::kBidirectionalTurnOnly, "BidirectionalTurnOnly"},
          {DirectionUsageRule::State::Type::kNoUse, "NoUse"},
          {DirectionUsageRule::State::Type::kParking, "Parking"}};
}

std::ostream& operator<<(std::ostream& out, const DirectionUsageRule::State::Type& type) {
  switch(type) {
    case DirectionUsageRule::State::Type::kWithS:
      out << "WithS";
      break;
    case DirectionUsageRule::State::Type::kAgainstS:
      out << "AgainstS";
      break;
    case DirectionUsageRule::State::Type::kBidirectional:
      out << "Bidirectional";
      break;
    case DirectionUsageRule::State::Type::kBidirectionalTurnOnly:
      out << "BidirectionalTurnOnly";
      break;
    case DirectionUsageRule::State::Type::kNoUse:
      out << "NoUse";
      break;
    case DirectionUsageRule::State::Type::kParking:
      out << "Parking";
  }
  return out;
}

std::ostream& operator<<(std::ostream& out, const DirectionUsageRule::State::Severity& severity) {
  switch(severity) {
    case DirectionUsageRule::State::Severity::kStrict:
      out << "Strict";
      break;
    case DirectionUsageRule::State::Severity::kPreferred:
      out << "Preferred";
      break;
  }
  return out;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
