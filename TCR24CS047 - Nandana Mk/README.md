## Author

Nandana MK

TCR24CS047
# Safe Semantic Planner

## Overview

Safe Semantic Planner is a Python-based path planning system inspired by LPA* concepts. It computes safe paths between a start state and a goal state while considering transition cost, safety, reliability, bad states, and dynamic updates.

## Features

- Safe path planning
- Bad state avoidance
- Dynamic transition updates
- Goal updates
- Transition addition
- Graph visualization
- Automatic PNG generation for test cases

## Technologies

- Python 3.10
- NetworkX
- Matplotlib

## Project Structure

```
SafeSemanticPlanner/
│
├── src/
│   ├── state.py
│   ├── transition.py
│   ├── planner.py
│   ├── utils.py
│   ├── test_cases.py
│   └── main.py
│
├── output/
├── requirements.txt
├── README.md
└── .gitignore
```

## Running

```bash
pip install -r requirements.txt
python src/main.py
```

## Output

Running the project generates:

- Console output for all six test cases
- Graph images 

