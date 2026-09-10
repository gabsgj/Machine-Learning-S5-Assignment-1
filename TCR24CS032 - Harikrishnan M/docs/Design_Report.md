# Design Report: Safe Semantic Planner

## State Representation
Cartesian state embeddings are mapped via unique `uint64_t` IDs.

## Data Structures
Hash maps (`std::unordered_map`) are used for g-values, rhs-values, and graph lookup tables. A `std::set` ordered by priority keys is used for priority queue management.

## Heuristic Function
A straight-line Euclidean distance heuristic is combined with adaptive distance penalties away from bad states.

## Safety Computation
Safety is calculated as the minimum Euclidean distance evaluated from candidate path states to the nearest bad state vector.

## Complexity Analysis
* **Time Complexity:** O(|E| log |V|) per search expansion.
* **Space Complexity:** O(|V| + |E|) for state graph storage.
