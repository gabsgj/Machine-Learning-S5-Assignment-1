cmake_minimum_required(VERSION 3.16)
project(SafeSemanticPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

file(GLOB SOURCES
    src/*.cpp
)

add_executable(SafeSemanticPlanner ${SOURCES})

# Target compiler flags
if (MSVC)
    target_compile_options(SafeSemanticPlanner PRIVATE /W4 /O2 /EHsc)
else()
    target_compile_options(SafeSemanticPlanner PRIVATE -Wall -Wextra -O3)
endif()
