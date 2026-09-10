cmake_minimum_required(VERSION 3.14)
project(SafeSemanticPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# Main library
add_library(PlannerLib src/DStarLite.cpp)

# Tests executable
add_executable(planner_tests tests/test_planner.cpp)
target_link_libraries(planner_tests PlannerLib)
