cmake_minimum_required(VERSION 3.14)
project(SafeSemanticPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(planner
    src/main.cpp
    src/planner.cpp
    src/test_runner.cpp
)

target_include_directories(planner PRIVATE src)
