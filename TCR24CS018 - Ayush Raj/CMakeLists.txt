cmake_minimum_required(VERSION 3.16)
project(quality_dstar_lite LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(quality_dstar_lite src/dstar_lite.cpp)
target_include_directories(quality_dstar_lite PUBLIC include)
target_compile_options(quality_dstar_lite PRIVATE -Wall -Wextra -Wpedantic)

add_executable(quality_dstar_demo src/main.cpp)
target_link_libraries(quality_dstar_demo PRIVATE quality_dstar_lite)

enable_testing()
add_executable(quality_dstar_tests tests/tests.cpp)
target_link_libraries(quality_dstar_tests PRIVATE quality_dstar_lite)
add_test(NAME quality_dstar_tests COMMAND quality_dstar_tests)
