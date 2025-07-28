cmake_minimum_required(VERSION 3.16)
project(MultiAgentPatrolSim VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags for development and release builds
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

# Find SFML graphics library
find_package(sfml-graphics QUIET)
if(NOT sfml-graphics_FOUND)
    # Fallback to pkg-config if CMake config not found
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(SFML REQUIRED sfml-graphics sfml-window sfml-system)
endif()

# Include directories
include_directories(src)

# Source files
set(SOURCES
    src/main.cpp
    src/grid.cpp
    src/agent.cpp
    src/behavior_tree.cpp
    src/pathfinding.cpp
    src/utils.cpp
    src/visualizer.cpp
    src/gui_visualizer.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME} Threads::Threads)

# Link SFML
if(sfml-graphics_FOUND)
    target_link_libraries(${PROJECT_NAME} sfml-graphics sfml-window sfml-system)
else()
    target_link_libraries(${PROJECT_NAME} ${SFML_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${SFML_INCLUDE_DIRS})
    target_compile_options(${PROJECT_NAME} PRIVATE ${SFML_CFLAGS_OTHER})
endif()

# Optional: Enable address sanitizer for debugging
option(ENABLE_ASAN "Enable AddressSanitizer" OFF)
if(ENABLE_ASAN)
    target_compile_options(${PROJECT_NAME} PRIVATE -fsanitize=address)
    target_link_options(${PROJECT_NAME} PRIVATE -fsanitize=address)
endif()

# Install target
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

# Print build configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
