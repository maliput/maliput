##############################################################################
# Sources
##############################################################################

add_library(plugin maliput_plugin.cc maliput_plugin_manager.cc)
add_library(maliput::plugin ALIAS plugin)

set_target_properties(plugin
  PROPERTIES
    OUTPUT_NAME maliput_plugin
)

target_link_libraries(plugin
  dl
  maliput::api
  maliput::common
  maliput::utility
)

target_include_directories(plugin
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

##############################################################################
# Export
##############################################################################

install(
  TARGETS plugin
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
