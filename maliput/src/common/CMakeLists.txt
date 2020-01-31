##############################################################################
# Sources
##############################################################################

set(COMMON_SOURCES
  filesystem.cc
  logger.cc
  maliput_abort_and_throw.cc
  maliput_logger.cc)

add_library(common ${COMMON_SOURCES})

target_include_directories(common
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(common
  PUBLIC
    drake::drake
)

##############################################################################
# Export
##############################################################################

install(
    TARGETS common
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(common)
