find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
add_subdirectory(api)
add_subdirectory(base)
add_subdirectory(common)
add_subdirectory(drake)
add_subdirectory(geometry_base)
add_subdirectory(math)
add_subdirectory(plugin)
add_subdirectory(routing)
add_subdirectory(utility)
