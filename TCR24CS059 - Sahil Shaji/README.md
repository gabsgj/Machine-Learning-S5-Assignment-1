# Safe Semantic Planner

Machine Learning Assignment 1

## Algorithm
D* Lite

## Features
- Safe path planning
- Bad state avoidance
- Dynamic replanning
- Goal updates
- Transition addition/removal
- Safety-aware cost function

## Build

g++ -std=c++17 -Iinclude src/DStarLite.cpp tests/test_planner.cpp -o planner_tests

## Run

./planner_tests
