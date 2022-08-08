##############################################################################
# Sources
##############################################################################

set(MATH_SOURCES
  axis_aligned_box.cc
  bounding_box.cc
  matrix.cc
  quaternion.cc
  overlapping_type.cc
  roll_pitch_yaw.cc
  saturate.cc
  vector.cc
)

add_library(math ${MATH_SOURCES})

add_library(maliput::math ALIAS math)

set_target_properties(math
  PROPERTIES
    OUTPUT_NAME maliput_math
)

target_include_directories(math
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(math
  PUBLIC
    maliput::common
)

##############################################################################
# Export
##############################################################################

install(
  TARGETS math
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
