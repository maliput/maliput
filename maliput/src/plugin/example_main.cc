#include "loader.h"
#include "plugin_base.h"

#include <gflags/gflags.h>

#include <dlfcn.h>
#include <iostream>
#include <string>

DEFINE_string(lib_name, "libmaliput_plugin_1.so", "Name of the .so file to be loaded.");

using namespace maliput::plugin;

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // The macro that this uses is provided as a compile definition in the
  // CMakeLists.txt file.
  const std::string PluginLibDir = MALIPUT_PLUGIN_LIBDIR;
  const std::string PluginLibName = FLAGS_lib_name;
  const std::string PluginLibPath = PluginLibDir + "/" + PluginLibName;

  std::cout << "PluginLibPath: " << PluginLibPath << std::endl;
  Loader<PluginBase> loader{PluginLibPath.c_str()};

  // create an instance of the class
  PluginBase* plugin_backend = loader.GetInstance();
  // use the class
  std::cout << "The backend is: " << plugin_backend->BackendName() << '\n' << std::endl;
  // destroy the class
  loader.DestroyInstance(plugin_backend);
}