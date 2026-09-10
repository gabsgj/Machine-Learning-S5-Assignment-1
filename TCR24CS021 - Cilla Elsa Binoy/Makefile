CXX := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Iinclude

SRCS := src/DStarLitePlanner.cpp src/main.cpp
BIN := planner

all: $(BIN)

$(BIN): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(BIN)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)

.PHONY: all run clean
