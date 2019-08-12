/// @file yaml_load.cc
///
/// Attempts to load a yaml file as input and build a multilane road geometry.
///
#include <iostream>
#include <string>

#include <gflags/gflags.h>

#include "maliput/api/road_geometry.h"
#include "maliput/common/logger.h"
#include "multilane/builder.h"
#include "multilane/loader.h"


namespace multilane = maliput::multilane;

DEFINE_string(yaml_file, "",
              "yaml input file defining a multilane road geometry");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_yaml_file.empty()) {
    maliput::log()->error("No input file!");
    return 1;
  }
  maliput::log()->info("Loading '{}'.", FLAGS_yaml_file);
  auto rg = multilane::LoadFile(multilane::BuilderFactory(), FLAGS_yaml_file);
  const std::vector<std::string> failures = rg->CheckInvariants();

  if (!failures.empty()) {
    for (const auto& f : failures) {
      maliput::log()->error(f);
    }
    return 1;
  }

  return 0;
}
