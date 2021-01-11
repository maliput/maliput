// Copyright 2021 Toyota Research Institute
#pragma once

#include <functional>
#include <memory>
#include <string>

#include <dlfcn.h>

#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/plugin/maliput_plugin_type.h"

namespace maliput {
namespace plugin {

/// MaliputPlugin loads a dynamic library.
/// After construction, the id and type of the plugin are provided.
/// I adition this allows you to execute any symbol from the library.
/// The library must contain at least two methods which they are:
/// @code{.cpp}
/// extern "C" std::string GetMaliputPluginId();
/// extern "C" MaliputPlugin::Type GetMaliputPluginType();
/// @endcode
class MaliputPlugin {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MaliputPlugin);

  /// Persistent identifier for a MaliputPlugin element.
  using Id = api::TypeSpecificIdentifier<class MaliputPlugin>;

  /// Types of plugin.
  enum class Type {
    kRoadNetworkLoader,
  };

  /// Constructs a MaliputPlugin
  /// @param path_to_lib Path to the library.
  /// @throws maliput::common::assertion_error When `path_to_lib` is empty.
  MaliputPlugin(const std::string& path_to_lib);

  MaliputPlugin() = delete;

  /// @returns The Id of the plugin.
  std::string GetId() const { return id_.string(); }

  /// @returns The MaliputPluginType of the plugin.
  MaliputPluginType GetType() const { return type_; }

  /// Finds and executes a symbol loaded by the plugin library.
  /// @tparam ReturnType The return type of the symbol that is executed.
  /// @tparam Args Types of the argument list.
  ///
  /// @param sym_name Name of the symbol to execute.
  /// @param args Argument list for the symbol call.
  /// @returns An object returned by the executed symbol.
  ///
  /// @throws maliput::common::assertion_error When `sym_name` is not found.
  template <typename ReturnType, typename... Args>
  ReturnType ExecuteSymbol(const std::string& sym_name, Args&&... args) const {
    // Reset error string.
    dlerror();
    typedef ReturnType (*method_t)(Args...);
    const method_t method = (method_t)dlsym(lib_handle_.get(), sym_name.c_str());
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
      MALIPUT_THROW_MESSAGE("Cannot load symbol " + sym_name + " : " + std::string(dlsym_error));
    }
    return method(std::forward<Args>(args)...);
  }

 private:
  /// Holds the symbol name that returns the id of the plugin.
  static constexpr char const* kMaliputPluginIdSym{"GetMaliputPluginId"};
  /// Holds the symbol name that returns the type of the plugin.
  static constexpr char const* kMaliputPluginTypeSym{"GetMaliputPluginType"};

  using LibraryHandleDeleter = std::function<void(void*)>;
  /// Handle of the library.
  std::unique_ptr<void, LibraryHandleDeleter> lib_handle_;
  /// Id of the plugin.
  MaliputPlugin::Id id_{"none"};
  /// MaliputPluginType of the plugin.
  MaliputPluginType type_;
};

}  // namespace plugin
}  // namespace maliput
