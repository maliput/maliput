# C++ Version
set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

# Compiler-specific C++17 activation.
if ("${CMAKE_CXX_COMPILER_ID} " MATCHES "GNU ")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if (NOT (GCC_VERSION VERSION_GREATER 6.9))
        message(FATAL_ERROR "${PROJECT_NAME} requires g++ 7.0 or greater.")
    endif ()
elseif ("${CMAKE_CXX_COMPILER_ID} " MATCHES "Clang ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
else ()
    message(FATAL_ERROR "Your C++ compiler does not support C++17.")
endif ()
