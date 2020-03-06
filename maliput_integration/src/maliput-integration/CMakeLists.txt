##############################################################################
# Sources
##############################################################################

add_executable(yaml_to_obj
  yaml_to_obj.cc)

ament_target_dependencies(yaml_to_obj
  "maliput"
  "maliput-utilities"
  "multilane"
)

target_link_libraries(yaml_to_obj
    gflags
    maliput-utilities::maliput-utilities
    multilane::multilane
    yaml-cpp
)

##############################################################################
# Install
##############################################################################

install(
    TARGETS yaml_to_obj
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
