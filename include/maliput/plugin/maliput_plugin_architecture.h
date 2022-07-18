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

/// @file maliput_plugin_architecture.h
/// @page maliput_plugin_architecture Maliput Plugin Architecture
/// @date October 7, 2021
/// @tableofcontents
///
/// @section maliput_plugin_architecture_section Maliput Plugin Architecture
///
/// @subsection plugin_architecture_overview Overview
///
/// This document aims to explain the plugin architecture that Maliput provides.
///
/// The main objective of the plugin architecture is to hand out an architecture that allows users to customize certain
/// systems implementations in an easy and effective way.
///
/// @subsection maliput_plugin MaliputPlugin
///
/// Maliput defines maliput::plugin::MaliputPlugin class which is in charge of providing an interface to interact with
/// a given dynamic library.
///
/// To be considered a maliput plugin, the following two functions must be defined:
/// @code{.cpp}
/// extern "C" char* GetMaliputPluginId();
/// extern "C" MaliputPluginType GetMaliputPluginType();
/// @endcode
///
/// @subsection maliput_plugin_types MaliputPlugin types
///
/// The `MaliputPlugin` type  that are currently supported are listed at the `enum` named
/// maliput::plugin::MaliputPluginType.
///
/// @subsection maliput_plugin_manager MaliputPluginManager
///
/// maliput::plugin::MaliputPluginManager manages the lifecycle of `MaliputPlugin`s.
/// It will try to load all the available plugins in the path(`MALIPUT_PLUGIN_PATH`) and make them available
/// via maliput::plugin::MaliputPluginManager::GetPlugin().
///
/// The MaliputPlugin's discovery process consists on retrieving all the MaliputPlugins that are located
/// as the `MALIPUT_PLUGIN_PATH` environment variable points to.
///
/// To extend the discovery process to other locations simply extend the `MALIPUT_PLUGIN_PATH` environment variable.
/// @code{.sh}
/// export MALIPUT_PLUGIN_PATH=/new/path/for/plugin/discovery:$MALIPUT_PLUGIN_PATH
/// @endcode
///
/// @section maliput_available_interfaces Maliput available interfaces
/// @subsection road_network_loader_plugin RoadNetworkLoader plugin
///
/// Maliput clients may opt to use the plugin architecture to load at runtime specific backends.
/// That simplifies the linkage process and reduces the number of compile time dependencies.
/// See maliput::plugin::RoadNetworkLoader class which offers a unified interface for maliput
/// users to load a maliput::api::RoadNetwork.
///
/// Maliput backend implementations must use `REGISTER_ROAD_NETWORK_LOADER_PLUGIN()` macro
/// to instantiate the necessary entry points of the plugin. Those symbols are required
/// by the plugin architecture discovery phase. Refer to the following code snippet for a
/// usage example:
///
/// @code{.cpp}
/// // Implementation of a maliput::plugin::RoadNetworkLoader using a custom maliput backend called `my_custom_backend`.
/// class RoadNetworkLoader : public maliput::plugin::RoadNetworkLoader {
///  public:
///   std::unique_ptr<const maliput::api::RoadNetwork> operator()(
///       const std::map<std::string, std::string>& properties) const override {
///     return my_custom_backend::loader::RoadNetworkLoader(properties)();
///   }
/// };
///
/// REGISTER_ROAD_NETWORK_LOADER_PLUGIN("my_custom_backend", RoadNetworkLoader);
/// @endcode
/// As it can be seen:
///  - `RoadNetworkLoader` class inherits from maliput::plugin::RoadNetworkLoader and the `operator()` calls into the
///  specific maliput backend load procedure.
///  - REGISTER_ROAD_NETWORK_LOADER_PLUGIN() macros is called.
///
/// Note: `MALIPUT_PLUGIN_PATH` must contain the path to the installed plugin shared library
/// in order to make maliput::plugin::MaliputPluginManager aware of its existence and load it.
///
/// @subsubsection using_custom_road_network_loader Using a custom RoadNetworkLoader plugin
///
/// After the creation of the maliput::plugin::MaliputPlugin that implements a maliput::plugin::RoadNetworkLoader and
/// the correct set up of the `MALIPUT_PLUGIN_PATH` discovery path, the use of this plugin is quite straightforward:
///
/// @code{.cpp}
/// const std::string plugin_name{"my_custom_backend"};
/// const std::map<std::string, std::string> loader_parameters{/* Parameters for the backend's builder if necessary */}
///
/// // Create maliput::plugin::MaliputPluginManager instance.
/// maliput::plugin::MaliputPluginManager manager;
///
/// // Get plugin.
/// const maliput::plugin::MaliputPlugin* maliput_plugin =
///     manager.GetPlugin(maliput::plugin::MaliputPlugin::Id(plugin_name));
///
/// // Verifies that the plugin was obtained.
/// if (!maliput_plugin) {
///   throw std::runtime_error("Plugin hasn't been found");
/// }
///
/// // Verifies plugin type.
/// if (maliput_plugin->GetType() != maliput::plugin::MaliputPluginType::kRoadNetworkLoader) {
///   throw std::runtime_error("Plugin type doesn't match");
/// }
///
/// // Obtains a pointer to an instance of the loader class.
/// auto rn_loader_ptr = maliput_plugin->ExecuteSymbol<maliput::plugin::RoadNetworkLoaderPtr>(
///         maliput::plugin::RoadNetworkLoader::GetEntryPoint());
///
/// // Use smart pointers to gracefully manage heap allocation.
/// std::unique_ptr<maliput::plugin::RoadNetworkLoader> road_network_loader{
///     reinterpret_cast<maliput::plugin::RoadNetworkLoader*>(rn_loader_ptr)};
///
/// // Generates the maliput::api::RoadNetwork.
/// std::unique_ptr<const maliput::api::RoadNetwork> road_network = (*road_network_loader)(loader_parameters);
///
///
/// @endcode
///
/// For convenience, maliput offers a helper method for loading a maliput::api::RoadNetwork from a plugin, which is a
/// shortcut to the above code snippet.
///
/// @code{.cpp}
/// const std::string road_network_loader_plugin_id{"my_custom_backend"};
/// const std::map<std::string, std::string> loader_parameters{/* Parameters for the backend's builder if necessary */}
///
/// // Create maliput::api::RoadNetwork instance.
/// std::unique_ptr<maliput::api::RoadNetwork> road_network =
/// maliput::plugin::CreateRoadNetwork(road_network_loader_plugin_id, loader_parameters);
///
/// @endcode
///
/// @section related_references Related References
///
/// - `maliput` backends that already have implemented their maliput::plugin::RoadNetworkLoader plugin:
///   - [`maliput_dragway`](https://github.com/ToyotaResearchInstitute/maliput_dragway)
///   - [`maliput_multilane`](https://github.com/ToyotaResearchInstitute/maliput_multilane)
///   - [`maliput_malidrive`](https://github.com/ToyotaResearchInstitute/maliput_malidrive)
///
/// - Python interface: There are python bindings for the plugin architecture, see
/// [`maliput_py`](https://github.com/ToyotaResearchInstitute/maliput_dragway) package for further information.
///
