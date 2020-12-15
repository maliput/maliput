##############################################################################
# Sources
##############################################################################

add_library(plugin_1 plugin_1.cc)

set_target_properties(plugin_1
  PROPERTIES
    OUTPUT_NAME maliput_plugin_1
)

target_include_directories(plugin_1
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)


add_library(plugin_2 plugin_2.cc)

set_target_properties(plugin_2
  PROPERTIES
    OUTPUT_NAME maliput_plugin_2
)

target_include_directories(plugin_2
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

##############################################################################
# Export
##############################################################################

install(
  TARGETS plugin_1 plugin_2
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION "${CMAKE_INSTALL_PREFIX}/plugin"
)

add_executable(plugin_test
               main.cc)

target_compile_definitions(plugin_test
PRIVATE
  MALIPUT_PLUGIN_LIBDIR="${CMAKE_INSTALL_PREFIX}/plugin")

target_link_libraries(plugin_test
  dl
)

install(
  TARGETS
  plugin_test
  DESTINATION "${CMAKE_INSTALL_PREFIX}/plugin"
)
