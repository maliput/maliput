##############################################################################
# Sources
##############################################################################
add_executable(dragway_to_urdf
  dragway_to_urdf.cc
)

ament_target_dependencies(dragway_to_urdf
  "dragway"
  "maliput"
  "maliput-utilities"
)

target_link_libraries(dragway_to_urdf
    dragway::dragway
    gflags
    maliput-utilities::maliput-utilities
    maliput::common
    maliput::geometry_base
)

##############################################################################
# Exports
##############################################################################

install(TARGETS dragway_to_urdf
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
