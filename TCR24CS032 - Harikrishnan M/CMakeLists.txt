cmake_minimum_required(VERSION 3.10)
project(SafeSemanticPlanner)

# Require C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable aggressive compiler warnings for code quality
add_compile_options(-Wall -Wextra -Wpedantic)

# Set default build type to Release for faster execution time
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Specify include directory
include_directories(include)

# Create the core library
add_library(planner_core 
    src/environment.cpp 
    src/heuristics.cpp 
    src/planner.cpp
)

# Main executable
add_executable(safe_semantic_planner src/main.cpp)
target_link_libraries(safe_semantic_planner planner_core)

# ==========================================
# TESTING CONFIGURATION (CTest)
# ==========================================
enable_testing()

# Test 1: Basic Reachability
add_executable(test_1 tests/test_case_1_reachability.cpp)
target_link_libraries(test_1 planner_core)
add_test(NAME ReachabilityTest COMMAND test_1)

# Test 2: Bad State Avoidance
add_executable(test_2 tests/test_case_2_bad_state.cpp)
target_link_libraries(test_2 planner_core)
add_test(NAME BadStateAvoidanceTest COMMAND test_2)

# Test 3: Safety Margin
add_executable(test_3 tests/test_case_3_safety_margin.cpp)
target_link_libraries(test_3 planner_core)
add_test(NAME SafetyMarginTest COMMAND test_3)

# Test 4: Dynamic Transition
add_executable(test_4 tests/test_case_4_dynamic_transition.cpp)
target_link_libraries(test_4 planner_core)
add_test(NAME DynamicTransitionTest COMMAND test_4)

# Test 5: Goal Update
add_executable(test_5 tests/test_case_5_goal_update.cpp)
target_link_libraries(test_5 planner_core)
add_test(NAME GoalUpdateTest COMMAND test_5)

# Test 6: Transition Addition
add_executable(test_6 tests/test_case_6_transition_addition.cpp)
target_link_libraries(test_6 planner_core)
add_test(NAME TransitionAdditionTest COMMAND test_6)