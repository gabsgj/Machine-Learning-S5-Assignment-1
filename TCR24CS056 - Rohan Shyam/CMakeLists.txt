cmake_minimum_required(VERSION 3.14)
project(SafeSemanticPlanner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# Core library
add_library(planner_lib STATIC
    planner.cpp
    planner.hpp
)
target_include_directories(planner_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Main executable (Demonstration of the 6 assignment scenarios)
add_executable(safe_planner main.cpp)
target_link_libraries(safe_planner PRIVATE planner_lib)

# Automated unit tests
add_executable(planner_tests tests.cpp)
target_link_libraries(planner_tests PRIVATE planner_lib)

# Test registration
enable_testing()
add_test(NAME AllTests COMMAND planner_tests)
