cmake_minimum_required(VERSION 3.5)
# Modified for the shared ROS1 Livox message package, 2026-10-03.
project(faster_lio)

set(DEFAULT_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# Use the workspace's shared ROS1 message package.

include(cmake/packages.cmake)

#definitions
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# options
option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)

if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()

add_subdirectory(src)
add_subdirectory(app)

option(HANDHELD_BUILD_TESTS "Build the MID360 preprocessing regression check" ON)
if(HANDHELD_BUILD_TESTS)
    add_executable(faster_mid360_preprocess_test
        ${PROJECT_SOURCE_DIR}/../../../../scripts/tests/faster_mid360_preprocess_test.cpp)
    target_link_libraries(faster_mid360_preprocess_test ${PROJECT_NAME} gflags)
    add_dependencies(faster_mid360_preprocess_test livox_ros_driver_gencpp)
endif()
