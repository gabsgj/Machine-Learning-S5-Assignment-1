cmake_minimum_required(VERSION 3.16)
project(SafeSemanticPlanner VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Include directories ──────────────────────────────────────────────
include_directories(${CMAKE_SOURCE_DIR}/include)

# ── Main executable ──────────────────────────────────────────────────
add_executable(safe_planner
    src/main.cpp
    src/d_star_lite_planner.cpp
    src/safety_utils.cpp
)

# ── Test executable ──────────────────────────────────────────────────
add_executable(test_planner
    tests/test_planner.cpp
    src/d_star_lite_planner.cpp
    src/safety_utils.cpp
)

# ── Compiler warnings ───────────────────────────────────────────────
if(MSVC)
    target_compile_options(safe_planner  PRIVATE /W4)
    target_compile_options(test_planner  PRIVATE /W4)
else()
    target_compile_options(safe_planner  PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(test_planner  PRIVATE -Wall -Wextra -Wpedantic)
endif()
