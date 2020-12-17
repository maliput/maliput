#pragma once

#include <dlfcn.h>
#include <memory>
#include <stdexcept>
#include <string>

#include "tools.h"

namespace maliput {
namespace plugin {

template <typename PluginBaseT>
class PluginLoader {
 public:
  PluginLoader(const std::string& file_name) {
    // load library
    lhandle = dlopen(GetPathToLib(file_name).c_str(), RTLD_LAZY);
    if (!lhandle) {
      throw std::runtime_error("Cannot load library: " + std::string(dlerror()));
    }
    // reset errors
    dlerror();

    // load the symbols
    creator = (create_t*)dlsym(lhandle, "LoadMaliputRoadNetwork");
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
      throw std::runtime_error("Cannot load symbol LoadMaliputRoadNetwork: " + std::string(dlsym_error));
    }
  }

  std::unique_ptr<PluginBaseT> GetInstance() { return creator(); }

  ~PluginLoader() { dlclose(lhandle); }

 private:
  // the types of the class factories
  typedef std::unique_ptr<PluginBaseT> create_t();

  create_t* creator;
  void* lhandle{nullptr};
};

}  // namespace plugin
}  // namespace maliput
