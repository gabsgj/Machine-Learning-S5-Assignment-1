CXX = g++
CXXFLAGS = -std=c++14 -O3 -Wall -Wextra -Iinclude

SRCS = src/LPAStarPlanner.cpp src/DStarLitePlanner.cpp src/main.cpp
TARGET = safe_planner.exe

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

clean:
	del /f $(TARGET)
