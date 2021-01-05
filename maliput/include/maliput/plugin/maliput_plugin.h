// Copyright 2021 Toyota Research Institute
#pragma once

#include <string>

#include <dlfcn.h>

#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"

namespace maliput {
namespace plugin {

/// Persistent identifier for a MaliputPlugin element.
using MaliputPluginId = api::TypeSpecificIdentifier<class MaliputPlugin>;

/// MaliputPlugin is constructed from a library handler previously loaded.
/// After construction, the id and type of the plugin are provided.
/// Besides, it helps to execute any symbol from the library.
class MaliputPlugin {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MaliputPlugin);

  /// Types of plugin.
  enum class Type {
    kRoadNetworkLoader,
  };

  /// Constructs a MaliputPlugin
  /// @param lib_handle library pointer.
  /// @throws maliput::common::assertion_error When `lib_handle` is nullptr.
  MaliputPlugin(void* lib_handle);

  MaliputPlugin() = delete;

  /// @returns The Id of the plugin.
  std::string GetId() { return id_.string(); }

  /// @returns The Type of the plugin.
  Type GetType() { return type_; }

  /// Finds and executes a symbol loaded by the plugin library.
  /// @tparam ReturnType The return type of the symbol that is executed.
  /// @param sym_name Name of the symbol to execute.
  /// @returns An object returned by the executed symbol.
  ///
  /// @throws maliput::common::assertion_error When `sym_name` is not found.
  template <typename ReturnType>
  ReturnType ExecuteSymbol(const std::string& sym_name) {
    dlerror();
    typedef ReturnType (*getter_t)();
    getter_t getter = (getter_t)dlsym(lib_handle_, sym_name.c_str());
    const char* dlsym_error = dlerror();
    if (dlsym_error) {
      const std::string msg{};
      MALIPUT_THROW_MESSAGE("Cannot load symbol " + sym_name + " : " + static_cast<std::string>(dlsym_error));
    }
    return getter();
  }

 private:
  /// Holds the symbol name that returns the id of the plugin.
  static constexpr char const* kMaliputPluginIdSym{"GetMaliputPluginId"};
  /// Holds the symbol name that returns the type of the plugin.
  static constexpr char const* kMaliputPluginTypeSym{"GetMaliputPluginType"};

  /// Handle of the library.
  void* lib_handle_;
  /// Id of the plugin.
  MaliputPluginId id_{"none"};
  /// Type of the plugin.
  Type type_;
};

}  // namespace plugin
}  // namespace maliput
