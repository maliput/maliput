##############################################################################
# Sources
##############################################################################

add_library(dumb_plugin_1 dumb_plugin_1.cc)
add_library(maliput::dumb_plugin_1 ALIAS dumb_plugin_1)

set_target_properties(dumb_plugin_1
  PROPERTIES
    OUTPUT_NAME maliput_dumb_plugin_1
)

ament_target_dependencies(dumb_plugin_1
    "maliput"
)

target_link_libraries(dumb_plugin_1
  maliput::plugin
)

##############################################################################
# Export
##############################################################################

set(TEST_PLUGIN_INSTALL_DIR
    "/tmp/maliput/test/plugins/")

install(
  TARGETS dumb_plugin_1
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
  LIBRARY DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
  RUNTIME DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
)
