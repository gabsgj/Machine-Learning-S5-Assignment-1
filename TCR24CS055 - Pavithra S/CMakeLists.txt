cmake_minimum_required(VERSION 3.10)
project(SafeSemanticPlanner CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(safe_planner src/main.cpp)
