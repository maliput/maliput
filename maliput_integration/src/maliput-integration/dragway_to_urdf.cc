/**
 * @file dragway_to_urdf.cc
 * Instantiates a dragway with a user-specified number of lanes and outputs
 * a URDF model of it.
 */
#include <limits>
#include <string>

#include <gflags/gflags.h>

#include "dragway/road_geometry.h"
#include "maliput-utilities/generate_urdf.h"
#include "maliput/api/road_geometry.h"
#include "maliput/common/filesystem.h"
#include "maliput/common/logger.h"
#include "maliput/common/maliput_abort.h"

DEFINE_int32(num_lanes, 2, "The number of lanes.");
DEFINE_double(length, 10, "The length of the dragway in meters.");
// By default, each lane is 3.7m (12 feet) wide, which is the standard used by
// the U.S. interstate highway system.
DEFINE_double(lane_width, 3.7, "The width of each lane in meters.");
// By default, the shoulder width is 3 m (10 feet) wide, which is the standard
// used by the U.S. interstate highway system.
DEFINE_double(shoulder_width, 3.0,
              "The width of the shoulders in meters. Both shoulders have the same "
              "width.");
DEFINE_double(maximum_height, 5.2, "The maximum modelled height above the road surface (meters).");
DEFINE_string(dirpath, ".",
              "The path to where the URDF and OBJ files should be saved. If this path "
              " does not exist, it is created.");
DEFINE_string(file_name_root, "dragway",
              "The root name of the files to create. For example, if the value of this "
              "parameter is \"foo\", the following files will be created: \"foo.urdf\", "
              "\"foo.obj\", and \"foo.mtl\". These files will be placed in the path "
              "specified by parameter 'dirpath'.");
DEFINE_string(spdlog_level, "unchanged",
              "sets the spdlog output threshold; possible values are "
              "'unchanged', "
              "'trace', "
              "'debug', "
              "'info', "
              "'warn', "
              "'err', "
              "'critical', "
              "'off'");

namespace maliput {
namespace integration {
namespace {

int exec(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  common::set_log_level(FLAGS_spdlog_level);

  const dragway::RoadGeometry road_geometry(
      api::RoadGeometryId{"Dragway with " + std::to_string(FLAGS_num_lanes) + " lanes."}, FLAGS_num_lanes, FLAGS_length,
      FLAGS_lane_width, FLAGS_shoulder_width, FLAGS_maximum_height, std::numeric_limits<double>::epsilon(),
      std::numeric_limits<double>::epsilon());

  utility::ObjFeatures features;

  // Creates the destination directory if it does not already exist.
  common::Path directory;
  directory.set_path(FLAGS_dirpath);
  if (!directory.exists()) {
    common::Filesystem::create_directory_recursive(directory);
  }
  MALIPUT_DEMAND(directory.exists());

  // The following is necessary for users to know where to find the resulting
  // files when this program is executed in a sandbox. This occurs, for example
  // when using `dragway_to_urdf`.
  const common::Path my_path = maliput::common::Filesystem::get_cwd();

  log()->info("Creating Dragway URDF in {}.", my_path.get_path());
  utility::GenerateUrdfFile(&road_geometry, directory.get_path(), FLAGS_file_name_root, features);
  return 0;
}

}  // namespace
}  // namespace integration
}  // namespace maliput

int main(int argc, char* argv[]) { return maliput::integration::exec(argc, argv); }
