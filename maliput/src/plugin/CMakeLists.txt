##############################################################################
# Sources
##############################################################################

add_library(plugin_loader plugin_loader.cc)
add_library(maliput::plugin_loader ALIAS plugin_loader)

set_target_properties(plugin_loader
  PROPERTIES
    OUTPUT_NAME maliput_plugin_loader
)

target_link_libraries(plugin_loader
  dl
  maliput::plugin_tools
)

target_include_directories(plugin_loader
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

###############################

add_library(plugin_tools tools.cc)

set_target_properties(plugin_tools
  PROPERTIES
    OUTPUT_NAME maliput_plugin_tools
)

add_library(maliput::plugin_tools ALIAS plugin_tools)

target_include_directories(plugin_tools
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_compile_definitions(plugin_tools
    PRIVATE
      MALIPUT_PLUGIN_LIBDIR="${CMAKE_INSTALL_PREFIX}/../plugins")

##############################################################################
# Export
##############################################################################

install(
  TARGETS plugin_tools plugin_loader
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
