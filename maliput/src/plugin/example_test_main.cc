#include "plugin_base.h"
#include "plugin_loader.h"

#include <gflags/gflags.h>

#include <dlfcn.h>
#include <iostream>
#include <memory>
#include <string>

DEFINE_string(lib_name, "libmaliput_plugin_1.so", "Name of the .so file to be loaded.");

using namespace maliput::plugin;

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  PluginLoader<PluginBase> loader{FLAGS_lib_name};

  // create an instance of the class
  std::unique_ptr<PluginBase> plugin_backend = loader.GetInstance();
  // use the class
  std::cout << "The backend is: " << plugin_backend->BackendName() << '\n' << std::endl;
}
