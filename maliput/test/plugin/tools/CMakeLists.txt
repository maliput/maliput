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

add_library(dumb_plugin_2 dumb_plugin_2.cc)
add_library(maliput::dumb_plugin_2 ALIAS dumb_plugin_2)

set_target_properties(dumb_plugin_2
  PROPERTIES
    OUTPUT_NAME maliput_dumb_plugin_2
)

ament_target_dependencies(dumb_plugin_2
    "maliput"
)

target_link_libraries(dumb_plugin_2
  maliput::plugin
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS dumb_plugin_1 dumb_plugin_2
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
  LIBRARY DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
  RUNTIME DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
)

##############################################################################
# Sources
##############################################################################

add_library(dumb_plugin_3 dumb_plugin_3.cc)
add_library(maliput::dumb_plugin_3 ALIAS dumb_plugin_3)

set_target_properties(dumb_plugin_3
  PROPERTIES
    OUTPUT_NAME maliput_dumb_plugin_3
)

ament_target_dependencies(dumb_plugin_3
    "maliput"
)

target_link_libraries(dumb_plugin_3
  maliput::plugin
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS  dumb_plugin_3
  EXPORT ${PROJECT_NAME}-targets
  # Intentionally this library is installed in a different location.
  ARCHIVE DESTINATION ${OTHER_TEST_PLUGIN_INSTALL_DIR}
  LIBRARY DESTINATION ${OTHER_TEST_PLUGIN_INSTALL_DIR}
  RUNTIME DESTINATION ${OTHER_TEST_PLUGIN_INSTALL_DIR}
)
