/**
 * Built-in Preset Test Scenarios for Interactive Web Visualizer
 * Corresponds exactly to Assignment Test Cases 1 through 6
 */

const PRESET_SCENARIOS = {
    testcase1: {
        title: "Test Case 1: Basic Reachability",
        desc: "Graph: S -> A -> B -> G. Expected unique valid path with minimal expansion.",
        problem: {
            initialState: 0,
            goalState: 3,
            badStates: [],
            states: [
                { id: 0, name: "S", embedding: [80, 250] },
                { id: 1, name: "A", embedding: [260, 250] },
                { id: 2, name: "B", embedding: [440, 250] },
                { id: 3, name: "G", embedding: [620, 250] }
            ],
            transitions: [
                { id: 1, from: 0, to: 1, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 2, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 3, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 0.99, available: true }
            ]
        }
    },
    testcase2: {
        title: "Test Case 2: Bad State Avoidance",
        desc: "Hazardous shortcut passes through bad state X. Safe bypass S -> C -> D -> G must be chosen with strictly 0 bad visits.",
        problem: {
            initialState: 0,
            goalState: 3,
            badStates: [4], // X is bad
            states: [
                { id: 0, name: "S", embedding: [80, 250] },
                { id: 1, name: "A", embedding: [240, 130] },
                { id: 4, name: "X [BAD]", embedding: [420, 130] },
                { id: 3, name: "G", embedding: [620, 250] },
                { id: 5, name: "C", embedding: [240, 370] },
                { id: 6, name: "D", embedding: [420, 370] }
            ],
            transitions: [
                { id: 1, from: 0, to: 1, cost: 1.0, safety: 1.0, reliability: 0.95, available: true },
                { id: 2, from: 1, to: 4, cost: 1.0, safety: 0.0, reliability: 0.95, available: true },
                { id: 3, from: 4, to: 3, cost: 1.0, safety: 0.0, reliability: 0.95, available: true },
                { id: 4, from: 0, to: 5, cost: 1.5, safety: 1.0, reliability: 0.98, available: true },
                { id: 5, from: 5, to: 6, cost: 1.5, safety: 1.0, reliability: 0.98, available: true },
                { id: 6, from: 6, to: 3, cost: 1.5, safety: 1.0, reliability: 0.98, available: true }
            ]
        }
    },
    testcase3: {
        title: "Test Case 3: Safety Margin Optimization",
        desc: "Two valid paths: Path 1 has lower raw cost but grazes danger state X. Path 2 provides wide spatial clearance.",
        problem: {
            initialState: 0,
            goalState: 3,
            badStates: [4],
            states: [
                { id: 0, name: "S", embedding: [80, 250] },
                { id: 1, name: "P1_1", embedding: [240, 220] },
                { id: 2, name: "P1_2", embedding: [420, 220] },
                { id: 3, name: "G", embedding: [620, 250] },
                { id: 4, name: "X [BAD]", embedding: [330, 160] },
                { id: 5, name: "P2_1", embedding: [240, 390] },
                { id: 6, name: "P2_2", embedding: [420, 390] }
            ],
            transitions: [
                { id: 1, from: 0, to: 1, cost: 1.0, safety: 0.4, reliability: 0.99, available: true },
                { id: 2, from: 1, to: 2, cost: 1.0, safety: 0.2, reliability: 0.99, available: true },
                { id: 3, from: 2, to: 3, cost: 1.0, safety: 0.4, reliability: 0.99, available: true },
                { id: 4, from: 0, to: 5, cost: 2.2, safety: 1.0, reliability: 0.99, available: true },
                { id: 5, from: 5, to: 6, cost: 2.2, safety: 1.0, reliability: 0.99, available: true },
                { id: 6, from: 6, to: 3, cost: 2.2, safety: 1.0, reliability: 0.99, available: true }
            ]
        }
    },
    testcase4: {
        title: "Test Case 4: Dynamic Transition Blockage",
        desc: "Initial optimal path is S -> A -> G. Click edge (A->G) or click 'Disable (A->G)' to test instant incremental LPA* rerouting!",
        problem: {
            initialState: 0,
            goalState: 3,
            badStates: [],
            states: [
                { id: 0, name: "S", embedding: [80, 250] },
                { id: 1, name: "A", embedding: [350, 140] },
                { id: 3, name: "G", embedding: [620, 250] },
                { id: 2, name: "B", embedding: [240, 360] },
                { id: 4, name: "C", embedding: [440, 360] }
            ],
            transitions: [
                { id: 1, from: 0, to: 1, cost: 1.5, safety: 1.0, reliability: 0.98, available: true },
                { id: 2, from: 1, to: 3, cost: 1.5, safety: 1.0, reliability: 0.98, available: true },
                { id: 3, from: 0, to: 2, cost: 2.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 4, from: 2, to: 4, cost: 2.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 5, from: 4, to: 3, cost: 2.0, safety: 1.0, reliability: 0.99, available: true }
            ]
        }
    },
    testcase5: {
        title: "Test Case 5: Dynamic Goal State Update",
        desc: "Initial goal G1. Click 'Switch to Goal G2' or click node G2 to observe LPA* goal shift replanning.",
        problem: {
            initialState: 0,
            goalState: 3,
            badStates: [],
            states: [
                { id: 0, name: "S", embedding: [80, 250] },
                { id: 1, name: "A", embedding: [230, 250] },
                { id: 2, name: "B", embedding: [380, 160] },
                { id: 3, name: "G1", embedding: [560, 140] },
                { id: 4, name: "C", embedding: [380, 340] },
                { id: 5, name: "G2", embedding: [560, 360] }
            ],
            transitions: [
                { id: 1, from: 0, to: 1, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 2, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 3, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 4, from: 1, to: 4, cost: 1.5, safety: 1.0, reliability: 0.99, available: true },
                { id: 5, from: 4, to: 5, cost: 1.5, safety: 1.0, reliability: 0.99, available: true }
            ]
        }
    },
    testcase6: {
        title: "Test Case 6: Dynamic Shortcut Insertion",
        desc: "Initial long path S -> A -> B -> C -> G (Cost 4.0). Click 'Insert Shortcut (A->G)' to watch incremental search instantly adapt!",
        problem: {
            initialState: 0,
            goalState: 4,
            badStates: [],
            states: [
                { id: 0, name: "S", embedding: [80, 250] },
                { id: 1, name: "A", embedding: [220, 250] },
                { id: 2, name: "B", embedding: [350, 250] },
                { id: 3, name: "C", embedding: [480, 250] },
                { id: 4, name: "G", embedding: [620, 250] }
            ],
            transitions: [
                { id: 1, from: 0, to: 1, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 2, from: 1, to: 2, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 3, from: 2, to: 3, cost: 1.0, safety: 1.0, reliability: 0.99, available: true },
                { id: 4, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 0.99, available: true }
            ]
        }
    },
    random_graph: {
        title: "Large Complex Semantic Mesh",
        desc: "Dense multi-hop Cartesian mesh with dynamic obstacles and multiple safe corridors.",
        generate: function() {
            const states = [];
            const transitions = [];
            const badStates = [4, 9, 13];
            const numStates = 16;
            const cols = 4;
            const rows = 4;

            for (let i = 0; i < numStates; ++i) {
                const c = i % cols;
                const r = Math.floor(i / cols);
                states.push({
                    id: i,
                    name: i === 0 ? "Start (0)" : (i === numStates - 1 ? "Goal (15)" : `S${i}`),
                    embedding: [80 + c * 170, 80 + r * 110]
                });
            }

            let tid = 1;
            for (let r = 0; r < rows; ++r) {
                for (let c = 0; c < cols; ++c) {
                    const u = r * cols + c;
                    // Right edge
                    if (c + 1 < cols) {
                        const v = r * cols + (c + 1);
                        transitions.push({ id: tid++, from: u, to: v, cost: 1.0, safety: 1.0, reliability: 0.98, available: true });
                    }
                    // Down edge
                    if (r + 1 < rows) {
                        const v = (r + 1) * cols + c;
                        transitions.push({ id: tid++, from: u, to: v, cost: 1.0, safety: 1.0, reliability: 0.98, available: true });
                    }
                    // Diagonal edge
                    if (c + 1 < cols && r + 1 < rows) {
                        const v = (r + 1) * cols + (c + 1);
                        transitions.push({ id: tid++, from: u, to: v, cost: 1.414, safety: 0.9, reliability: 0.95, available: true });
                    }
                }
            }

            return {
                initialState: 0,
                goalState: 15,
                badStates,
                states,
                transitions
            };
        }
    }
};
