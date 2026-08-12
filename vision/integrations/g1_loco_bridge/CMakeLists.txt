cmake_minimum_required(VERSION 3.16)
project(g1_loco_audit_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(UNITREE_SDK_ROOT "/home/unitree/unitree_sdk2-main" CACHE PATH
    "Path to the robot-local unitree_sdk2 source tree")
if(NOT EXISTS "${UNITREE_SDK_ROOT}/CMakeLists.txt")
  message(FATAL_ERROR "unitree_sdk2 not found at ${UNITREE_SDK_ROOT}")
endif()

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory("${UNITREE_SDK_ROOT}" "${CMAKE_BINARY_DIR}/unitree_sdk2"
                 EXCLUDE_FROM_ALL)

add_executable(g1_loco_audit_bridge g1_loco_audit_bridge.cpp)
target_include_directories(g1_loco_audit_bridge PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(g1_loco_audit_bridge PRIVATE unitree_sdk2)

add_executable(audit_command_test audit_command_test.cpp)
target_include_directories(audit_command_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")

enable_testing()
add_test(NAME audit_command_test COMMAND audit_command_test)
