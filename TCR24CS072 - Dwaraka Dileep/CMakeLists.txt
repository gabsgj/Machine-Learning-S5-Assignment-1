cmake_minimum_required(VERSION 3.10)
project(SafeSemanticPlanner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

include_directories(${CMAKE_SOURCE_DIR}/include)

# Core planner library (reused by the demo, the tests, and the experiments).
add_library(planner_core src/LPAStar.cpp)

# Demo CLI.
add_executable(planner_demo src/main.cpp)
target_link_libraries(planner_demo PRIVATE planner_core)

# Automated test suite.
add_executable(planner_tests tests/test_planner.cpp)
target_link_libraries(planner_tests PRIVATE planner_core)

# Experimental evaluation harness.
add_executable(run_experiments experiments/run_experiments.cpp)
target_link_libraries(run_experiments PRIVATE planner_core)

enable_testing()
add_test(NAME planner_tests COMMAND planner_tests)
