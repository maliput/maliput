##############################################################################
# Sources
##############################################################################

set(MATH_SOURCES
  matrix.cc
  quaternion.cc
  roll_pitch_yaw.cc
  saturate.cc
  vector.cc)

add_library(math ${MATH_SOURCES})

target_include_directories(math
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(math
  PUBLIC
    common
)

##############################################################################
# Export
##############################################################################

install(
    TARGETS math
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(math)
