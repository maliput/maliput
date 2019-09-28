#include "maliput/api/lane.h"

namespace maliput {
namespace api {

// These instantiations must match the API documentation in lane.h.
template <>
GeoPositionT<double> Lane::ToGeoPositionT<double>(const LanePositionT<double>& lane_pos) const {
  return DoToGeoPosition(lane_pos);
}

template <>
GeoPositionT<drake::AutoDiffXd> Lane::ToGeoPositionT<drake::AutoDiffXd>(
    const LanePositionT<drake::AutoDiffXd>& lane_pos) const {
  // Fail fast if lane_pos contains derivatives of inconsistent sizes.
  const Eigen::VectorXd deriv = lane_pos.s().derivatives();
  MALIPUT_THROW_UNLESS(deriv.size() == lane_pos.r().derivatives().size());
  MALIPUT_THROW_UNLESS(deriv.size() == lane_pos.h().derivatives().size());

  return DoToGeoPositionAutoDiff(lane_pos);
}

template <>
GeoPositionT<drake::symbolic::Expression> Lane::ToGeoPositionT<drake::symbolic::Expression>(
    const LanePositionT<drake::symbolic::Expression>& lane_pos) const {
  return DoToGeoPositionSymbolic(lane_pos);
}

template <>
LanePositionResultT<double> Lane::ToLanePositionT<double>(const GeoPositionT<double>& geo_pos) const {
  return DoToLanePosition(geo_pos);
}

template <>
LanePositionResultT<drake::AutoDiffXd> Lane::ToLanePositionT<drake::AutoDiffXd>(
    const GeoPositionT<drake::AutoDiffXd>& geo_pos) const {
  // Fail fast if geo_pos contains derivatives of inconsistent sizes.
  const Eigen::VectorXd deriv = geo_pos.x().derivatives();
  MALIPUT_THROW_UNLESS(deriv.size() == geo_pos.y().derivatives().size());
  MALIPUT_THROW_UNLESS(deriv.size() == geo_pos.z().derivatives().size());

  LanePositionResultT<drake::AutoDiffXd> result = DoToLanePositionAutoDiff(geo_pos);

  // If the partial derivatives of result, nearest_point and distance are not of
  // the same dimension as those in geo_pos, pad them with zeros of the same
  // dismension as those in geo_pos.
  drake::Vector3<drake::AutoDiffXd> srh = result.lane_position.srh();
  Eigen::internal::make_coherent(srh.x().derivatives(), deriv);
  Eigen::internal::make_coherent(srh.y().derivatives(), deriv);
  Eigen::internal::make_coherent(srh.z().derivatives(), deriv);
  result.lane_position.set_srh(srh);

  drake::Vector3<drake::AutoDiffXd> xyz = result.nearest_position.xyz();
  Eigen::internal::make_coherent(xyz.x().derivatives(), deriv);
  Eigen::internal::make_coherent(xyz.y().derivatives(), deriv);
  Eigen::internal::make_coherent(xyz.z().derivatives(), deriv);
  result.nearest_position.set_xyz(xyz);

  Eigen::internal::make_coherent(result.distance.derivatives(), deriv);

  return result;
}

template <>
LanePositionResultT<drake::symbolic::Expression> Lane::ToLanePositionT<drake::symbolic::Expression>(
    const GeoPositionT<drake::symbolic::Expression>& geo_pos) const {
  return DoToLanePositionSymbolic(geo_pos);
}

}  // namespace api
}  // namespace maliput
