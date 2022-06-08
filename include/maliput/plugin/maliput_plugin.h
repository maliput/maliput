// BSD 3-Clause License
//
// Copyright (c) 2022, Woven Planet. All rights reserved.
// Copyright (c) 2021-2022, Toyota Research Institute. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <dlfcn.h>

#include <functional>
#include <memory>
#include <string>

#include "maliput/api/type_specific_identifier.h"
#include "maliput/common/maliput_copyable.h"
#include "maliput/plugin/maliput_plugin_type.h"

namespace maliput {
namespace plugin {

/// MaliputPlugin loads a dynamic library.
///
/// After construction, the id and type of the plugin are provided.
/// Via MaliputPlugin::ExecuteSymbol() functions can be run from the library.
/// It is thought to be the entry point of the plugin.
/// Specific plugin implementations would specialize the template invocation with the
/// necessary types so the functionality for the plugin can be achieved.
///
/// To be considered a maliput plugin, the following two functions must be defined:
/// @code{.cpp}
/// extern "C" char* GetMaliputPluginId();
/// extern "C" MaliputPluginType GetMaliputPluginType();
/// @endcode
///
/// @note When shared library and executable are compiled using `ubsan`(undefined behavior sanitizer)
///       the property `ENABLE_EXPORTS` should be enabled on the executable target in order
///       to instruct the linker to add all symbols to the dynamic symbol table.
///       For further information see next
///       [link](https://stackoverflow.com/questions/57361776/use-ubsan-with-dynamically-loaded-shared-libraries).
class MaliputPlugin {
 public:
  MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(MaliputPlugin);

  /// Persistent identifier for a MaliputPlugin element.
  using Id = api::TypeSpecificIdentifier<class MaliputPlugin>;

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
  // Holds the symbol name that returns the id of the plugin.
  static constexpr char const* kMaliputPluginIdSym{"GetMaliputPluginId"};
  // Holds the symbol name that returns the type of the plugin.
  static constexpr char const* kMaliputPluginTypeSym{"GetMaliputPluginType"};

  // Functor that closes the DL library.
  struct DLHandleDeleter {
    void operator()(void* dl_handle) { dlclose(dl_handle); }
  };

  // Handle of the library.
  std::unique_ptr<void, DLHandleDeleter> lib_handle_;
  // Id of the plugin.
  MaliputPlugin::Id id_{"none"};
  // MaliputPluginType of the plugin.
  MaliputPluginType type_;
};

}  // namespace plugin
}  // namespace maliput
