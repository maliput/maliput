// Copyright 2021 Toyota Research Institute
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
/// It will try to load all the available plugins in path and made them available
/// via maliput::plugin::MaliputPluginManager::GetPlugin().
///
/// The MaliputPlugin's discovery process consists on retrieve all the MaliputPlugin that are located
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
/// maliput::plugin::RoadNetworkLoader is an interface that allows the user to provide a custom
/// maliput::plugin::RoadNetworkLoader functor. By doing so, a custom implementation of the maliput::api::RoadNetwork
/// can be dinamically linked to maliput in runtime.
///
/// A `REGISTER_ROAD_NETWORK_LOADER_PLUGIN()` macro is provided to easily register the plugin and add the methods that
/// are necessary to convert the dynamic library into a maliput::plugin::MaliputPlugin.
///
/// As an example:
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
///  - maliput::plugin::RoadNetworkLoader class is inherited and the `operator()` is overriden returning an
///  maliput::api::RoadNetwork implemented using a custom backend.
///  - REGISTER_ROAD_NETWORK_LOADER_PLUGIN() macros is called.
///
/// Note: In order to be able to be loaded by the MaliputPluginManager the `MALIPUT_PLUGIN_PATH`'s paths must contain
/// the location of the installed library created from above example code.
///
/// @subsubsection using_custom_road_network_loader Using custom RoadNetworkLoader plugin
///
/// After the creation of the maliput::plugin::MaliputPlugin that implements a maliput::plugin::RoadNetworkLoader and
/// the correct set-up of the MALIPUT_PLUGIN_PATH discovery path, the use of this plugin is quite straight forward:
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
/// maliput::plugin::RoadNetworkLoaderPtr rn_loader_ptr =
///     maliput_plugin->ExecuteSymbol<maliput::plugin::RoadNetworkLoaderPtr>(
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
