##############################################################################
# Sources
##############################################################################

set(COMMON_SOURCES
  filesystem.cc
  logger.cc
  maliput_abort_and_throw.cc)

add_library(common ${COMMON_SOURCES})

add_library(maliput::common ALIAS common)

set_target_properties(common
  PROPERTIES
    OUTPUT_NAME maliput_common
)

target_include_directories(common
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(common
  PRIVATE
    fmt::fmt
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS common
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
