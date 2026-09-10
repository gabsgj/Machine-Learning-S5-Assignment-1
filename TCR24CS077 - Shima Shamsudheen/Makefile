CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Iinclude

all: safe_planner_test experiment bonus_tests

safe_planner_test: src/LPAStarPlanner.cpp src/main.cpp include/*.h
	$(CXX) $(CXXFLAGS) src/LPAStarPlanner.cpp src/main.cpp -o safe_planner_test

experiment: src/LPAStarPlanner.cpp src/experiment.cpp include/*.h
	$(CXX) $(CXXFLAGS) src/LPAStarPlanner.cpp src/experiment.cpp -o experiment

bonus_tests: src/LPAStarPlanner.cpp src/bonus_tests.cpp include/*.h
	$(CXX) $(CXXFLAGS) src/LPAStarPlanner.cpp src/bonus_tests.cpp -o bonus_tests

clean:
	rm -f safe_planner_test experiment bonus_tests *.o *.exe

.PHONY: all clean
