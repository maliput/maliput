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

###############################

add_library(plugin_2 plugin_2.cc)

set_target_properties(plugin_2
  PROPERTIES
    OUTPUT_NAME maliput_plugin_2
)

target_include_directories(plugin_2
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
      MALIPUT_PLUGIN_LIBDIR="${CMAKE_INSTALL_PREFIX}/plugin")

###############################

add_executable(example_test
  example_test_main.cc)

target_link_libraries(example_test
  dl
  gflags
  maliput::plugin_tools
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS plugin_1 plugin_2 plugin_tools example_test
  EXPORT ${PROJECT_NAME}-targets
  DESTINATION "${CMAKE_INSTALL_PREFIX}/plugin"
)
