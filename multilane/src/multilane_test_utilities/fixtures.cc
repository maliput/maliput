#include "multilane_test_utilities/fixtures.h"

#include <limits>
#include <string>

#include "maliput/api/lane.h"
#include "maliput/common/filesystem.h"
#include "multilane/builder.h"
#include "multilane/loader.h"

using maliput::api::LaneId;
using maliput::multilane::BuilderFactory;
using maliput::multilane::LoadFile;

namespace maliput {
namespace multilane {

constexpr char MULTILANE_RESOURCE_VAR[] = "MULTILANE_RESOURCE_ROOT";

BranchAndMergeBasedTest::BranchAndMergeBasedTest()
    : road_geometry_(
          LoadFile(
            BuilderFactory(),
            maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_VAR) +
            "/branch_and_merge.yaml")),
      index_(road_geometry_->ById()),
      total_length_(index_.GetLane(LaneId("l:1.1_0"))->length() +
                    index_.GetLane(LaneId("l:1.2_0"))->length() +
                    index_.GetLane(LaneId("l:1.3_0"))->length()) {}

LoopBasedTest::LoopBasedTest()
    : road_geometry_(
          LoadFile(
            BuilderFactory(),
            maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_VAR) +
            "/loop.yaml")),
      index_(road_geometry_->ById()) {}

MultiBranchBasedTest::MultiBranchBasedTest()
    : road_geometry_(LoadFile(
          BuilderFactory(),
          maliput::common::Filesystem::get_env_path(MULTILANE_RESOURCE_VAR) +
          "/multi_branch.yaml")),
      index_(road_geometry_->ById()) {}

}  // namespace multilane
}  // namespace maliput
