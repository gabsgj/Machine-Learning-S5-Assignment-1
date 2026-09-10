# Safe Semantic Planner

A web-based Safe Semantic Planner that uses the **LPA* (Lifelong Planning A*) algorithm** to find safe and efficient paths through a state space.

## Project Overview

The Safe Semantic Planner represents a planning problem using states and transitions. Each transition has a cost, while states can have safety values. The planner searches for a path from the initial state to the goal state while considering both cost and safety.

The project consists of a web-based frontend and a C++ backend planner.

## Features

- Interactive web-based user interface
- State-space representation
- Initial state and goal state configuration
- State safety values
- Transition costs
- Safe path detection
- LPA* path planning algorithm
- Total path cost calculation
- Minimum safety score calculation
- Planning result display
- C++ backend server

## Technologies Used

- HTML
- CSS
- JavaScript
- C++
- LPA* Algorithm
- MSYS2 UCRT64

## Project Structure

    SafeSemanticPlanner/
    │
    ├── frontend/
    │   └── index.html
    │
    ├── backend/
    │   ├── State.h
    │   ├── Transition.h
    │   ├── PlanningProblem.h
    │   ├── PlanningResult.h
    │   ├── Planner.h
    │   ├── LPAStarPlanner.h
    │   ├── LPAStarPlanner.cpp
    │   ├── main.cpp
    │   └── server.cpp
    │
    └── README.md

## Backend

The backend is implemented in C++ and contains the planning logic.

The planner uses:

- States
- Transitions
- Transition costs
- Safety scores
- Initial state
- Goal state

The LPA* planner calculates a suitable path from the initial state to the goal state.

## Example Planning Result

    Safe path found!

    State Path: 1 -> 2 -> 4
    Total Cost: 7
    Minimum Safety: 0.9
    Transitions Used: 1 2

## How to Compile the Backend

Open the **MSYS2 UCRT64** terminal and navigate to the backend folder:

    cd /c/Users/SHOBIN/Downloads/SafeSemanticPlanner/backend

Compile the backend:

    g++ -std=c++17 server.cpp LPAStarPlanner.cpp -o server.exe -lws2_32

Run the server:

    ./server.exe

The server is designed to run at:

    http://127.0.0.1:8080

## How to Run the Frontend

Open the project in **Visual Studio Code**.

Open the `frontend/index.html` file and run it using **Live Server**.

The frontend communicates with the C++ backend to execute the planner and display the planning results.

## Development Status

The Safe Semantic Planner frontend, state-space model, LPA* planner, and C++ backend have been implemented. Frontend-backend integration is currently being refined and tested.

## Future Improvements

- Dynamic state and transition creation
- Improved visualization of the state graph
- Real-time environment updates
- Better safety constraint handling
- Additional planning algorithms
- Improved frontend-backend communication
- Deployment of the planner as a web application

## Author

**SHOBIN P N**

## License

This project is developed for academic and educational purposes.
