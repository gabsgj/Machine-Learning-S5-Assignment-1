const fs = require('fs');

// Import LPAPlanner logic from app.js equivalent
class LPAPlanner {
    static INF = Infinity;
    static EPS = 1e-9;

    static KeyLess(a, b) {
        if (Math.abs(a.k1 - b.k1) > LPAPlanner.EPS) return a.k1 < b.k1;
        return a.k2 < b.k2;
    }

    static euclideanDist(a, b) {
        let sum = 0;
        for (let i = 0; i < Math.min(a.length, b.length); i++) {
            sum += (a[i] - b[i]) * (a[i] - b[i]);
        }
        return Math.sqrt(sum);
    }

    plan(problem, alpha = 2.0) {
        const statesMap = new Map();
        const badStatesSet = new Set(problem.badStates);
        const succs = new Map();
        const preds = new Map();
        const g = new Map();
        const rhs = new Map();
        const safetyDist = new Map();

        const s_start = Number(problem.initialState);
        const s_goal = Number(problem.goalState);

        for (const s of problem.states) {
            statesMap.set(Number(s.id), s);
            succs.set(Number(s.id), []);
            preds.set(Number(s.id), []);
        }

        for (const s of problem.states) {
            const sid = Number(s.id);
            g.set(sid, LPAPlanner.INF);
            rhs.set(sid, LPAPlanner.INF);

            if (badStatesSet.size === 0 || badStatesSet.has(sid)) {
                safetyDist.set(sid, 0.0);
            } else {
                let minDist = LPAPlanner.INF;
                for (const bId of badStatesSet) {
                    const badStateObj = statesMap.get(Number(bId));
                    if (badStateObj) {
                        const dist = LPAPlanner.euclideanDist(s.embedding, badStateObj.embedding);
                        minDist = Math.min(minDist, dist);
                    }
                }
                safetyDist.set(sid, minDist);
            }
        }

        for (const t of problem.transitions) {
            const u = Number(t.from);
            const v = Number(t.to);
            if (succs.has(u)) succs.get(u).push({ transId: t.id, to: v, cost: Number(t.cost), available: Boolean(t.available) });
            if (preds.has(v)) preds.get(v).push({ transId: t.id, to: u, cost: Number(t.cost), available: Boolean(t.available) });
        }

        const heuristic = (u) => {
            const uObj = statesMap.get(u);
            const gObj = statesMap.get(s_goal);
            if (!uObj || !gObj) return 0.0;
            return LPAPlanner.euclideanDist(uObj.embedding, gObj.embedding);
        };

        const calculateKey = (u) => {
            const min_val = Math.min(g.get(u), rhs.get(u));
            return { k1: min_val + heuristic(u), k2: min_val };
        };

        let openList = [];
        const openListInsert = (u, k) => openList.push({ key: k, id: u });
        const openListRemove = (u) => { openList = openList.filter(item => item.id !== u); };
        const openListContains = (u) => openList.some(item => item.id === u);

        const openListTopKey = () => {
            if (openList.length === 0) return { k1: LPAPlanner.INF, k2: LPAPlanner.INF };
            let minItem = openList[0];
            for (let i = 1; i < openList.length; i++) {
                if (LPAPlanner.KeyLess(openList[i].key, minItem.key)) minItem = openList[i];
            }
            return minItem.key;
        };

        const openListPop = () => {
            let minIndex = 0;
            for (let i = 1; i < openList.length; i++) {
                if (LPAPlanner.KeyLess(openList[i].key, openList[minIndex].key)) minIndex = i;
            }
            const node = openList[minIndex].id;
            openList.splice(minIndex, 1);
            return node;
        };

        const getEffectiveCost = (e) => {
            if (!e.available || badStatesSet.has(e.to)) return LPAPlanner.INF;
            const distToBad = safetyDist.get(e.to) || 0.0;
            const safetyFactor = (distToBad > 0.0) ? (alpha / distToBad) : 0.0;
            return e.cost + safetyFactor;
        };

        const updateVertex = (u) => {
            if (u !== s_start) {
                rhs.set(u, LPAPlanner.INF);
                const pList = preds.get(u) || [];
                for (const p of pList) {
                    const c = getEffectiveCost(p);
                    if (c < LPAPlanner.INF && g.get(p.to) < LPAPlanner.INF) {
                        rhs.set(u, Math.min(rhs.get(u), g.get(p.to) + c));
                    }
                }
            }
            if (openListContains(u)) openListRemove(u);
            if (Math.abs(g.get(u) - rhs.get(u)) > LPAPlanner.EPS) openListInsert(u, calculateKey(u));
        };

        const t0 = process.hrtime.bigint();
        rhs.set(s_start, 0.0);
        openListInsert(s_start, calculateKey(s_start));

        let iterations = 0;
        while (openList.length > 0) {
            const topKey = openListTopKey();
            const goalKey = calculateKey(s_goal);
            if (!LPAPlanner.KeyLess(topKey, goalKey) && Math.abs(rhs.get(s_goal) - g.get(s_goal)) <= LPAPlanner.EPS) break;

            const u = openListPop();
            iterations++;
            if (g.get(u) > rhs.get(u)) {
                g.set(u, rhs.get(u));
                for (const s of (succs.get(u) || [])) updateVertex(s.to);
            } else {
                g.set(u, LPAPlanner.INF);
                updateVertex(u);
                for (const s of (succs.get(u) || [])) updateVertex(s.to);
            }
        }
        const t1 = process.hrtime.bigint();

        const result = {
            success: false,
            statePath: [],
            totalCost: 0.0,
            safetyScore: 0.0,
            executionTimeMs: Number(t1 - t0) / 1e6,
            iterations
        };

        if ((g.get(s_goal) || LPAPlanner.INF) >= LPAPlanner.INF) return result;

        let curr = s_goal;
        result.statePath.push(curr);
        let minSafety = safetyDist.get(curr) || 0.0;

        while (curr !== s_start) {
            let bestPrev = curr;
            let minCost = LPAPlanner.INF;
            for (const p of (preds.get(curr) || [])) {
                const c = getEffectiveCost(p);
                const gVal = g.get(p.to);
                if (c < LPAPlanner.INF && gVal < LPAPlanner.INF) {
                    if (gVal + c < minCost) {
                        minCost = gVal + c;
                        bestPrev = p.to;
                    }
                }
            }
            if (bestPrev === curr) { result.success = false; return result; }
            result.totalCost += minCost - g.get(bestPrev);
            curr = bestPrev;
            result.statePath.push(curr);
            minSafety = Math.min(minSafety, safetyDist.get(curr) || 0.0);
        }

        result.statePath.reverse();
        result.success = true;
        result.safetyScore = minSafety;
        return result;
    }
}

// Generate Grid Benchmark Graph
function generateGridProblem(rows, cols, badRatio = 0.1) {
    const states = [];
    const transitions = [];
    const badStates = [];
    let stateId = 1;
    let transId = 1000;

    const grid = [];
    for (let r = 0; r < rows; r++) {
        grid[r] = [];
        for (let c = 0; c < cols; c++) {
            grid[r][c] = stateId;
            states.push({ id: stateId, embedding: [c * 100, r * 100] });
            if (Math.random() < badRatio && !(r === 0 && c === 0) && !(r === rows - 1 && c === cols - 1)) {
                badStates.push(stateId);
            }
            stateId++;
        }
    }

    for (let r = 0; r < rows; r++) {
        for (let c = 0; c < cols; c++) {
            const u = grid[r][c];
            // Right neighbor
            if (c + 1 < cols) {
                const v = grid[r][c + 1];
                transitions.push({ id: transId++, from: u, to: v, cost: 1.0, safety: 1.0, reliability: 1.0, available: true });
                transitions.push({ id: transId++, from: v, to: u, cost: 1.0, safety: 1.0, reliability: 1.0, available: true });
            }
            // Down neighbor
            if (r + 1 < rows) {
                const v = grid[r + 1][c];
                transitions.push({ id: transId++, from: u, to: v, cost: 1.0, safety: 1.0, reliability: 1.0, available: true });
                transitions.push({ id: transId++, from: v, to: u, cost: 1.0, safety: 1.0, reliability: 1.0, available: true });
            }
        }
    }

    return {
        initialState: grid[0][0],
        goalState: grid[rows - 1][cols - 1],
        badStates,
        states,
        transitions
    };
}

const planner = new LPAPlanner();

// Experiment 1: Alpha Tradeoff
console.log("=== EXPERIMENT 1: Safety Penalty Alpha Trade-off ===");
const problemC = {
    initialState: 1,
    goalState: 4,
    badStates: [3],
    states: [
        { id: 1, embedding: [100, 250] },
        { id: 2, embedding: [300, 100] },
        { id: 3, embedding: [300, 400] }, // Bad state
        { id: 4, embedding: [500, 250] }
    ],
    transitions: [
        { id: 101, from: 1, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 102, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 103, from: 1, to: 2, cost: 2.0, safety: 1.0, reliability: 1.0, available: true },
        { id: 104, from: 2, to: 4, cost: 2.0, safety: 1.0, reliability: 1.0, available: true }
    ]
};

[0.0, 1.0, 2.0, 5.0, 10.0].forEach(alpha => {
    const res = planner.plan(problemC, alpha);
    console.log(`Alpha=${alpha.toFixed(1)} -> Path: [${res.statePath.join('->')}] | Cost: ${res.totalCost.toFixed(2)} | MinSafetyDist: ${res.safetyScore.toFixed(2)}`);
});

// Experiment 2: Graph Scaling Benchmarks
console.log("\n=== EXPERIMENT 2: Scaling Analysis (Grid Networks) ===");
const sizes = [
    { r: 5, c: 5 },    // 25 states
    { r: 10, c: 10 },  // 100 states
    { r: 20, c: 20 },  // 400 states
    { r: 50, c: 50 }   // 2500 states
];

sizes.forEach(({ r, c }) => {
    const gridProb = generateGridProblem(r, c, 0.15);
    const res = planner.plan(gridProb, 2.5);
    console.log(`Grid ${r}x${c} (${gridProb.states.length} states, ${gridProb.transitions.length} edges) -> Success: ${res.success} | Time: ${res.executionTimeMs.toFixed(3)} ms | Iterations: ${res.iterations} | Cost: ${res.totalCost.toFixed(2)}`);
});
