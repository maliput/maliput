#include "plugin_base.h"

#include <dlfcn.h>
#include <iostream>
#include <string>

using namespace maliput::plugin;

/////////////////////////////////////////////////
// The macro that this uses is provided as a compile definition in the
// CMakeLists.txt file.
const std::string PluginLibDir = MALIPUT_PLUGIN_LIBDIR;

const std::string PluginLibName = "libmaliput_plugin_2.so";
const std::string PluginLibPath = PluginLibDir + "/" + PluginLibName;
int main() {
  std::cout << "PluginLibDir: " << PluginLibDir << std::endl;
  // load the triangle library
  void* plugin = dlopen(PluginLibPath.c_str(), RTLD_LAZY);
  if (!plugin) {
    std::cerr << "Cannot load library: " << dlerror() << '\n';
    return 1;
  }
  // reset errors
  dlerror();
  // load the symbols
  create_t* Instanciate = (create_t*)dlsym(plugin, "create");
  const char* dlsym_error = dlerror();

  if (dlsym_error) {
    std::cerr << "Cannot load symbol create: " << dlsym_error << '\n';
    return 1;
  }
  destroy_t* destroy_plugin = (destroy_t*)dlsym(plugin, "destroy");
  dlsym_error = dlerror();
  if (dlsym_error) {
    std::cerr << "Cannot load symbol destroy: " << dlsym_error << '\n';
    return 1;
  }
  // create an instance of the class
  PluginBase* plugin_backend = Instanciate();
  // use the class
  std::cout << "The backend is: " << plugin_backend->BackendName() << '\n' << std::endl;
  // destroy the class
  destroy_plugin(plugin_backend);
  // unload the plugin library
  dlclose(plugin);
}