#pragma once

#include <dlfcn.h>
#include <memory>
#include <stdexcept>
#include <string>

#include "maliput/common/maliput_throw.h"
#include "tools.h"

namespace maliput {
namespace plugin {

/// Loads a library and looks for a factory method in it in order to get an instance
/// of `PluginBaseT`.
/// @tparam PluginBaseT Type of the instance object will be created.
template <typename PluginBaseT>
class PluginLoader {
 public:
  /// Constructs a PluginLoader.
  /// @param file_name Name of the dynamic library file.
  /// @param factory_method Name of the method, which is not mangled by the compiler, and is located within the target
  /// library.
  ///
  /// @throws maliput::common::assertion_error When `file_name` library can't be loaded.
  /// @throws maliput::common::assertion_error When `factory_method` symbol can't be found.
  PluginLoader(const std::string& file_name, const std::string& factory_method) {
    // Load library.
    lhandle_ = dlopen(GetPathToLib(file_name).c_str(), RTLD_LAZY);
    if (!lhandle_) {
      const std::string msg{"Cannot load library: " + static_cast<std::string>(dlerror())};
      MALIPUT_THROW_MESSAGE(msg);
    }
    dlerror();

    // Load the symbol.
    creator_ = (create_t_*)dlsym(lhandle_, factory_method.c_str());
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
      const std::string msg{"Cannot load symbol " + factory_method + ": " + static_cast<std::string>(dlsym_error)};
      MALIPUT_THROW_MESSAGE(msg);
    }
  }

  /// Returns an instance of an object of type PluginBaseT.
  std::unique_ptr<PluginBaseT> GetInstance() const { return creator_(); }

  /// Close library from destructor.
  ~PluginLoader() { dlclose(lhandle_); }

 private:
  // Holds the name of the function to find out in the library.

  typedef std::unique_ptr<PluginBaseT> create_t_();
  // Pointer to the factory method within the library.
  create_t_* creator_;
  // Handler of the loaded library.
  void* lhandle_{nullptr};
};

}  // namespace plugin
}  // namespace maliput
