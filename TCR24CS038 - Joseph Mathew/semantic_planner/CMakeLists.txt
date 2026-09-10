cmake_minimum_required(VERSION 3.10)
project(semantic_planner)
set(CMAKE_CXX_STANDARD 17)

add_executable(semantic_planner
    src/main.cpp
    src/LPAStarPlanner.cpp
)

target_include_directories(semantic_planner PRIVATE src)
