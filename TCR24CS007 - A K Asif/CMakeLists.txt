cmake_minimum_required(VERSION 3.15)
project(SafeSemanticPlanner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(planner
    src/main.cpp
    src/Planner.cpp
)

target_include_directories(planner PRIVATE src)
