/**
 * Test case data — mirrors the 6 test cases from tests/run_tests.cpp plus an
 * interactive sandbox environment for live bad-state toggling and rerouting.
 */

export const testCases = [
  {
    id: 0,
    name: "Sandbox: Branching Paths",
    description: "Dual independent routes (S→A1→A2→G vs S→B1→B2→G) for live bad-state rerouting",
    proves: "Live bad-state toggling and dynamic path rerouting",
    isSandbox: true,
    problem: {
      initialState: 1,
      goalState: 6,
      badStates: [],
      states: [
        { id: 1, embedding: [0, 1] },   // Start (S)
        { id: 2, embedding: [1, 2] },   // Route A - State A1
        { id: 3, embedding: [2, 2] },   // Route A - State A2
        { id: 4, embedding: [1, 0] },   // Route B - State B1
        { id: 5, embedding: [2, 0] },   // Route B - State B2
        { id: 6, embedding: [3, 1] },   // Goal (G)
      ],
      transitions: [
        // Route A (Top) — total cost 3.0
        { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 3, from: 3, to: 6, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        // Route B (Bottom) — total cost 3.6
        { id: 4, from: 1, to: 4, cost: 1.2, safety: 1.0, reliability: 1.0, available: true },
        { id: 5, from: 4, to: 5, cost: 1.2, safety: 1.0, reliability: 1.0, available: true },
        { id: 6, from: 5, to: 6, cost: 1.2, safety: 1.0, reliability: 1.0, available: true },
      ],
    },
    replanAction: null,
  },
  {
    id: 1,
    name: "Basic Reachability",
    description: "Finds the unique valid path S→A→B→G",
    proves: "Finds the unique valid path",
    problem: {
      initialState: 1,
      goalState: 4,
      badStates: [],
      states: [
        { id: 1, embedding: [0, 0] },
        { id: 2, embedding: [1, 0] },
        { id: 3, embedding: [2, 0] },
        { id: 4, embedding: [3, 0] },
      ],
      transitions: [
        { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 3, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
      ],
    },
    replanAction: null,
  },
  {
    id: 2,
    name: "Bad State Avoidance",
    description: "Never routes through a bad state, even if that path exists",
    proves: "Never routes through a bad state",
    problem: {
      initialState: 1,
      goalState: 5,
      badStates: [3],
      states: [
        { id: 1, embedding: [0, 0] },
        { id: 2, embedding: [1, 0] },
        { id: 3, embedding: [2, 0] },
        { id: 4, embedding: [0, 1] },
        { id: 5, embedding: [2, 1] },
      ],
      transitions: [
        { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 3, from: 3, to: 5, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 4, from: 1, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 5, from: 4, to: 5, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
      ],
    },
    replanAction: null,
  },
  {
    id: 3,
    name: "Safety Margin Trade-off",
    description: "Picks a costlier path when it stays farther from danger",
    proves: "Balances cost against distance from bad states",
    problem: {
      initialState: 1,
      goalState: 4,
      badStates: [5],
      states: [
        { id: 1, embedding: [0, 0] },
        { id: 2, embedding: [1, 0.1] },
        { id: 3, embedding: [1, 3] },
        { id: 4, embedding: [2, 0] },
        { id: 5, embedding: [1, 0] },
      ],
      transitions: [
        { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 2, from: 2, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 3, from: 1, to: 3, cost: 3.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 4, from: 3, to: 4, cost: 3.0, safety: 1.0, reliability: 1.0, available: true },
      ],
    },
    replanAction: null,
  },
  {
    id: 4,
    name: "Dynamic Transition Removal",
    description: "Replans around a disabled edge (A→G becomes unavailable)",
    proves: "Replans around a disabled edge",
    problem: {
      initialState: 1,
      goalState: 3,
      badStates: [],
      states: [
        { id: 1, embedding: [0, 0] },
        { id: 2, embedding: [1, 0] },
        { id: 3, embedding: [2, 0] },
      ],
      transitions: [
        { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 3, from: 1, to: 3, cost: 5.0, safety: 1.0, reliability: 1.0, available: true },
      ],
    },
    replanAction: {
      type: "disableTransition",
      label: "Disable edge A→G (transition 2)",
      transitionId: 2,
      fromState: 2,
      toState: 3,
    },
  },
  {
    id: 5,
    name: "Goal Update",
    description: "Replans for a new goal without a full rebuild",
    proves: "Efficient incremental replanning on goal change",
    problem: {
      initialState: 1,
      goalState: 3,
      badStates: [],
      states: [
        { id: 1, embedding: [0, 0] },
        { id: 2, embedding: [1, 0] },
        { id: 3, embedding: [2, 0] },
        { id: 4, embedding: [3, 0] },
      ],
      transitions: [
        { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 3, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
      ],
    },
    replanAction: {
      type: "changeGoal",
      label: "Change goal to state 4",
      newGoal: 4,
    },
  },
  {
    id: 6,
    name: "Transition Addition",
    description: "Discovers a new shortcut once available (direct S→G)",
    proves: "Finds a shorter path after edge addition",
    problem: {
      initialState: 1,
      goalState: 4,
      badStates: [],
      states: [
        { id: 1, embedding: [0, 0] },
        { id: 2, embedding: [1, 0] },
        { id: 3, embedding: [2, 0] },
        { id: 4, embedding: [3, 0] },
      ],
      transitions: [
        { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 3, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
      ],
    },
    replanAction: {
      type: "addTransition",
      label: "Add shortcut S→G (cost 1.5)",
      transition: {
        id: 4, from: 1, to: 4, cost: 1.5, safety: 1.0, reliability: 1.0, available: true,
      },
    },
  },
];
