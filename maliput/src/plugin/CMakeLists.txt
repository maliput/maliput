##############################################################################
# Sources
##############################################################################

add_library(plugin_loader plugin_loader.cc tools.cc)
add_library(maliput::plugin_loader ALIAS plugin_loader)

set_target_properties(plugin_loader
  PROPERTIES
    OUTPUT_NAME maliput_plugin_loader
)

target_link_libraries(plugin_loader
  dl
  maliput::common
)

target_include_directories(plugin_loader
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_definitions(plugin_loader
  PRIVATE
    MALIPUT_PLUGIN_LIBDIR="${CMAKE_INSTALL_PREFIX}/../plugins")

###############################

add_library(plugin load_road_network.cc)
add_library(maliput::plugin ALIAS plugin)

set_target_properties(plugin
  PROPERTIES
    OUTPUT_NAME maliput_plugin
)

target_link_libraries(plugin
  maliput::plugin_loader
)

target_include_directories(plugin
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

##############################################################################
# Export
##############################################################################

install(
  TARGETS plugin plugin_loader
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
