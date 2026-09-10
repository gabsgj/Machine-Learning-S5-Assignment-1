/**
 * Pre-configured Scenarios and Test Cases for the Safe Semantic Planner
 */

const SCENARIOS = {
    "tc1": {
        name: "Test Case 1: Basic Reachability",
        description: "Linear graph S -> A -> B -> G. Verifies standard graph reachability and optimal sequential transition execution.",
        initialState: 1,
        goalState: 4,
        badStates: [],
        states: [
            { id: 1, embedding: [100, 300], label: "S (Start)" },
            { id: 2, embedding: [300, 300], label: "A" },
            { id: 3, embedding: [500, 300], label: "B" },
            { id: 4, embedding: [700, 300], label: "G (Goal)" }
        ],
        transitions: [
            { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 3, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true }
        ]
    },

    "tc2": {
        name: "Test Case 2: Bad State Avoidance",
        description: "Two potential paths: Unsafe lower-cost path passes through Bad State X. Safe detour bypasses X with zero hazard visits.",
        initialState: 1,
        goalState: 4,
        badStates: [3],
        states: [
            { id: 1, embedding: [100, 300], label: "S (Start)" },
            { id: 2, embedding: [300, 160], label: "A (Unsafe)" },
            { id: 3, embedding: [500, 160], label: "X [BAD STATE]" },
            { id: 4, embedding: [700, 300], label: "G (Goal)" },
            { id: 5, embedding: [300, 440], label: "C (Safe Detour)" },
            { id: 6, embedding: [500, 440], label: "D (Safe Detour)" }
        ],
        transitions: [
            { id: 1, from: 1, to: 2, cost: 1.0, safety: 0.2, reliability: 0.90, available: true },
            { id: 2, from: 2, to: 3, cost: 1.0, safety: 0.0, reliability: 0.90, available: true },
            { id: 3, from: 3, to: 4, cost: 1.0, safety: 0.0, reliability: 0.90, available: true },
            { id: 4, from: 1, to: 5, cost: 1.3, safety: 1.0, reliability: 0.99, available: true },
            { id: 5, from: 5, to: 6, cost: 1.4, safety: 1.0, reliability: 0.99, available: true },
            { id: 6, from: 6, to: 4, cost: 1.3, safety: 1.0, reliability: 0.99, available: true }
        ]
    },

    "tc3": {
        name: "Test Case 3: Safety Margin & Tradeoff",
        description: "Path 1 has minimal cost (2.0) but skirts tight near Bad State B. Path 2 is wider with higher cost (3.5) but maximizes clearance margin D.",
        initialState: 1,
        goalState: 4,
        badStates: [3],
        states: [
            { id: 1, embedding: [100, 300], label: "S (Start)" },
            { id: 2, embedding: [400, 240], label: "A (Close to Bad)" },
            { id: 3, embedding: [400, 150], label: "B [OBSTACLE]" },
            { id: 4, embedding: [700, 300], label: "G (Goal)" },
            { id: 5, embedding: [250, 480], label: "C (Wide Clearance)" },
            { id: 6, embedding: [550, 480], label: "D (Wide Clearance)" }
        ],
        transitions: [
            { id: 1, from: 1, to: 2, cost: 1.0, safety: 0.5, reliability: 0.90, available: true },
            { id: 2, from: 2, to: 4, cost: 1.0, safety: 0.5, reliability: 0.90, available: true },
            { id: 3, from: 1, to: 5, cost: 1.2, safety: 1.0, reliability: 0.99, available: true },
            { id: 4, from: 5, to: 6, cost: 1.1, safety: 1.0, reliability: 0.99, available: true },
            { id: 5, from: 6, to: 4, cost: 1.2, safety: 1.0, reliability: 0.99, available: true }
        ]
    },

    "tc4": {
        name: "Test Case 4: Dynamic Transition Failure",
        description: "Initial path S -> A -> G is optimal. When edge (A, G) becomes unavailable, LPA* incrementally replans to alternative route S -> B -> C -> G.",
        initialState: 1,
        goalState: 3,
        badStates: [],
        states: [
            { id: 1, embedding: [100, 300], label: "S (Start)" },
            { id: 2, embedding: [400, 160], label: "A (Primary)" },
            { id: 3, embedding: [700, 300], label: "G (Goal)" },
            { id: 4, embedding: [300, 450], label: "B (Bypass)" },
            { id: 5, embedding: [500, 450], label: "C (Bypass)" }
        ],
        transitions: [
            { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 3, from: 1, to: 4, cost: 1.2, safety: 1.0, reliability: 1.0, available: true },
            { id: 4, from: 4, to: 5, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 5, from: 5, to: 3, cost: 1.2, safety: 1.0, reliability: 1.0, available: true }
        ]
    },

    "tc5": {
        name: "Test Case 5: Dynamic Goal Update",
        description: "Goal state shifts from G1 to G2 during runtime. LPA* preserves valid g-values and re-routes search tree efficiently.",
        initialState: 1,
        goalState: 3,
        badStates: [],
        states: [
            { id: 1, embedding: [100, 300], label: "S (Start)" },
            { id: 2, embedding: [350, 180], label: "Hub 1" },
            { id: 3, embedding: [650, 160], label: "G1 (Old Goal)" },
            { id: 4, embedding: [350, 420], label: "Hub 2" },
            { id: 5, embedding: [700, 440], label: "G2 (New Goal)" }
        ],
        transitions: [
            { id: 1, from: 1, to: 2, cost: 1.1, safety: 1.0, reliability: 1.0, available: true },
            { id: 2, from: 2, to: 3, cost: 1.1, safety: 1.0, reliability: 1.0, available: true },
            { id: 3, from: 1, to: 4, cost: 1.1, safety: 1.0, reliability: 1.0, available: true },
            { id: 4, from: 4, to: 5, cost: 1.6, safety: 1.0, reliability: 1.0, available: true },
            { id: 5, from: 2, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true }
        ]
    },

    "tc6": {
        name: "Test Case 6: Transition Addition (Shortcut)",
        description: "Standard path S -> A -> B -> G (Cost 3.0). A new shortcut edge A -> G (Cost 1.05) is inserted, instantly discovered by LPA*.",
        initialState: 1,
        goalState: 4,
        badStates: [],
        states: [
            { id: 1, embedding: [100, 300], label: "S (Start)" },
            { id: 2, embedding: [300, 300], label: "A" },
            { id: 3, embedding: [500, 300], label: "B" },
            { id: 4, embedding: [700, 300], label: "G (Goal)" }
        ],
        transitions: [
            { id: 1, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 2, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 3, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true }
        ]
    },

    "kg": {
        name: "Bonus: Semantic Knowledge Graph",
        description: "Vector embedding space of concepts. Bypasses untrusted/poisoned knowledge entity to reach verified insight.",
        initialState: 1,
        goalState: 6,
        badStates: [3],
        states: [
            { id: 1, embedding: [120, 300], label: "Concept_Query" },
            { id: 2, embedding: [280, 200], label: "Entity_Resolver" },
            { id: 3, embedding: [480, 180], label: "Poisoned_Concept [BAD]" },
            { id: 4, embedding: [350, 420], label: "Knowledge_Fusion" },
            { id: 5, embedding: [560, 420], label: "Verified_Fact" },
            { id: 6, embedding: [720, 300], label: "Goal_Insight" }
        ],
        transitions: [
            { id: 1, from: 1, to: 2, cost: 1.2, safety: 0.9, reliability: 0.98, available: true },
            { id: 2, from: 2, to: 3, cost: 0.8, safety: 0.1, reliability: 0.50, available: true },
            { id: 3, from: 3, to: 6, cost: 0.9, safety: 0.1, reliability: 0.50, available: true },
            { id: 4, from: 2, to: 4, cost: 1.4, safety: 0.95, reliability: 0.99, available: true },
            { id: 5, from: 4, to: 5, cost: 1.1, safety: 0.99, reliability: 0.99, available: true },
            { id: 6, from: 5, to: 6, cost: 1.0, safety: 0.99, reliability: 0.99, available: true }
        ]
    },

    "drone": {
        name: "Bonus: Urban Autonomous Drone Corridor",
        description: "Multi-obstacle city flight path with no-fly hazard zones, dynamic crosswinds, and high-reliability safe corridors.",
        initialState: 1,
        goalState: 10,
        badStates: [4, 7],
        states: [
            { id: 1, embedding: [80, 300], label: "Port Alpha (Start)" },
            { id: 2, embedding: [200, 160], label: "WayPt N1" },
            { id: 3, embedding: [200, 440], label: "WayPt S1" },
            { id: 4, embedding: [360, 200], label: "No-Fly Zone 1 [BAD]" },
            { id: 5, embedding: [360, 340], label: "Central Corridor" },
            { id: 6, embedding: [360, 480], label: "South Bypass" },
            { id: 7, embedding: [520, 360], label: "Radar Storm [BAD]" },
            { id: 8, embedding: [520, 180], label: "North Express" },
            { id: 9, embedding: [520, 480], label: "South Express" },
            { id: 10, embedding: [720, 300], label: "Port Beta (Goal)" }
        ],
        transitions: [
            { id: 1, from: 1, to: 2, cost: 1.8, safety: 0.95, reliability: 0.99, available: true },
            { id: 2, from: 1, to: 3, cost: 1.8, safety: 0.95, reliability: 0.99, available: true },
            { id: 3, from: 2, to: 4, cost: 1.2, safety: 0.0, reliability: 0.40, available: true },
            { id: 4, from: 2, to: 8, cost: 2.5, safety: 0.95, reliability: 0.98, available: true },
            { id: 5, from: 3, to: 5, cost: 1.5, safety: 0.90, reliability: 0.95, available: true },
            { id: 6, from: 3, to: 6, cost: 1.6, safety: 0.99, reliability: 0.99, available: true },
            { id: 7, from: 5, to: 7, cost: 1.1, safety: 0.0, reliability: 0.30, available: true },
            { id: 8, from: 6, to: 9, cost: 1.7, safety: 0.99, reliability: 0.99, available: true },
            { id: 9, from: 8, to: 10, cost: 2.1, safety: 0.95, reliability: 0.99, available: true },
            { id: 10, from: 9, to: 10, cost: 2.2, safety: 0.99, reliability: 0.99, available: true },
            { id: 11, from: 5, to: 8, cost: 1.9, safety: 0.85, reliability: 0.92, available: true }
        ]
    }
};
