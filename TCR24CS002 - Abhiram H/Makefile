CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Iinclude
SRC := src/main.cpp src/DStarLitePlanner.cpp
BIN := planner_demo

.PHONY: all clean run test

all: $(BIN)

$(BIN): $(SRC) include/*.hpp
	$(CXX) $(CXXFLAGS) $(SRC) -o $(BIN)

run: $(BIN)
	./$(BIN)

test: run

clean:
	rm -f $(BIN) results.csv
