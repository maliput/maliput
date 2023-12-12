add_subdirectory(common)
add_subdirectory(systems)

add_library(drake
  arc_length_integrator.cc
  inverse_arc_length_integrator.cc
)

add_library(maliput::drake ALIAS drake)

set_target_properties(
  drake
  PROPERTIES
    OUTPUT_NAME ${PROJECT_NAME}_drake
)

target_include_directories(
  drake
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(drake
  PUBLIC
    maliput::common
    maliput::math
  PRIVATE
    dl
    Eigen3::Eigen
    maliput::drake_common
    maliput::drake_common_trajectories
    maliput::drake_systems_analysis
    maliput::drake_systems_framework
)

install(
  TARGETS drake
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
