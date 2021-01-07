##############################################################################
# Sources
##############################################################################

set(SOURCES
  filesystem.cc
)
add_library(utility ${SOURCES})

add_library(maliput::utility ALIAS utility)

set_target_properties(utility
  PROPERTIES
    OUTPUT_NAME maliput_utility
)
target_link_libraries(utility
  stdc++fs
)

target_include_directories(utility
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

##############################################################################
# Export
##############################################################################

install(
  TARGETS utility
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(utilities)
