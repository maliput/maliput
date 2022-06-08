##############################################################################
# Sources
##############################################################################

add_library(multiply_integers_test_plugin multiply_integers_test_plugin.cc)
add_library(maliput::multiply_integers_test_plugin ALIAS multiply_integers_test_plugin)

set_target_properties(multiply_integers_test_plugin
  PROPERTIES
    OUTPUT_NAME maliput_multiply_integers_test_plugin
)

target_link_libraries(multiply_integers_test_plugin
  maliput::plugin
)

add_library(sum_integers_test_plugin sum_integers_test_plugin.cc)
add_library(maliput::sum_integers_test_plugin ALIAS sum_integers_test_plugin)

set_target_properties(sum_integers_test_plugin
  PROPERTIES
    OUTPUT_NAME maliput_sum_integers_test_plugin
)

target_link_libraries(sum_integers_test_plugin
  maliput::plugin
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS multiply_integers_test_plugin sum_integers_test_plugin
  ARCHIVE DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
  LIBRARY DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
  RUNTIME DESTINATION ${TEST_PLUGIN_INSTALL_DIR}
)

##############################################################################
# Sources
##############################################################################

add_library(lorem_ipsum_test_plugin lorem_ipsum_test_plugin.cc)
add_library(maliput::lorem_ipsum_test_plugin ALIAS lorem_ipsum_test_plugin)

set_target_properties(lorem_ipsum_test_plugin
  PROPERTIES
    OUTPUT_NAME maliput_lorem_ipsum_test_plugin
)

target_link_libraries(lorem_ipsum_test_plugin
  maliput::plugin
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS  lorem_ipsum_test_plugin
  # Intentionally this library is installed in a different location.
  ARCHIVE DESTINATION ${OTHER_TEST_PLUGIN_INSTALL_DIR}
  LIBRARY DESTINATION ${OTHER_TEST_PLUGIN_INSTALL_DIR}
  RUNTIME DESTINATION ${OTHER_TEST_PLUGIN_INSTALL_DIR}
)
