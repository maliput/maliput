##############################################################################
# Sources
##############################################################################

set(BASE_SOURCES
  intersection.cc
  intersection_book.cc
  manual_phase_provider.cc
  manual_rule_state_provider.cc
  phase_based_rule_state_provider.cc
  phase_ring_book_loader.cc
  road_rulebook_loader.cc
  manual_phase_ring_book.cc
  manual_rulebook.cc
  traffic_light_book.cc
  traffic_light_book_loader.cc
  )

add_library(base ${BASE_SOURCES})
target_include_directories(base
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(base
  PUBLIC
    api
    drake::drake)

##############################################################################
# Tests
##############################################################################

##############################################################################
# Export
##############################################################################

include(CMakePackageConfigHelpers)

install(
    TARGETS base
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

ament_export_libraries(base)
