##############################################################################
# Project
##############################################################################

cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(maliput LANGUAGES C CXX VERSION 3.0.0)

option(BUILD_TESTS "Build test programs" OFF)

##############################################################################
# Find 3rd Party Packages
##############################################################################

message(STATUS "\n\n====== Finding 3rd Party Packages ======\n")

find_package(ament_cmake REQUIRED)
find_package(drake REQUIRED)

##############################################################################
# Project Configuration
##############################################################################

message(STATUS "\n\n========= Project Configuration ========\n")

set(BUILD_SHARED_LIBS true)

add_subdirectory(include)
add_subdirectory(src)

##############################################################################
# Tests
##############################################################################

if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()


##############################################################################
# Export
##############################################################################

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(drake)
ament_export_interfaces(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_package()
