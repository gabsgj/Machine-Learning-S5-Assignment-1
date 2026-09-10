cmake_minimum_required(VERSION 3.16)
project(safe_semantic_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(planner
    src/planner.cpp
)

target_include_directories(planner PUBLIC include)

if (MSVC)
    target_compile_options(planner PRIVATE /W4)
else()
    target_compile_options(planner PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(safe_planner src/main.cpp)
target_link_libraries(safe_planner PRIVATE planner)

add_executable(planner_tests tests/test_planner.cpp)
target_link_libraries(planner_tests PRIVATE planner)

enable_testing()
add_test(NAME planner_tests COMMAND planner_tests)
