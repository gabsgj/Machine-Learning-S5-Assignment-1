# User Manual: Safe Semantic Planner

## System Requirements
* C++17 compliant compiler (`g++` >= 7.0 or `clang++` >= 5.0)
* CMake >= 3.10
* GNU Make & CTest

## Build Instructions
1. Open terminal in the project root directory.
2. Create and navigate into the build folder:
   ```bash
   mkdir -p build && cd build
cmake ..
make 
ctest --output-on-failure
./test_1  # Replaces '1' with test case numbers 1 through 6
./safe_semantic_planner
