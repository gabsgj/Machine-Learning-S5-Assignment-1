CXX = g++
CXXFLAGS = -std=c++14 -O3 -Wall -Wextra -Iinclude

TARGET = safe_planner
SRC = src/main.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	@mkdir -p visualizer
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET) $(TARGET).exe visualizer/results.json

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
