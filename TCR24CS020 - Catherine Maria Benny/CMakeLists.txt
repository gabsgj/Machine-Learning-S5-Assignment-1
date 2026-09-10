cmake_minimum_required(VERSION 3.10)
project(SafePath CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

include_directories(include)

set(CORE_SOURCES
    src/State.cpp
    src/Transition.cpp
    src/DStarLite.cpp
    src/Metrics.cpp
    src/GraphGenerator.cpp
    src/VisualisationData.cpp
    src/TestScenarios.cpp
)

# Main CLI executable: ./safepath --test N | --demo | --experiment
add_executable(safepath src/main.cpp ${CORE_SOURCES})

# Unit test executable: ./test_runner
add_executable(test_runner tests/test_cases.cpp ${CORE_SOURCES})

enable_testing()
add_test(NAME UnitTests COMMAND test_runner)
