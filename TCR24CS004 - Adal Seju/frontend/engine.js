/**
 * Safe Semantic Planner - In-Browser LPA* & D* Lite Engine
 * PCCST503 Machine Learning - Assignment 1
 * Mirror of C++ Core Engine
 */

const INF = Infinity;
const EPSILON = 1e-9;

class Key {
    constructor(k1 = INF, k2 = INF) {
        this.k1 = k1;
        this.k2 = k2;
    }

    lessThan(other) {
        if (Math.abs(this.k1 - other.k1) > EPSILON) {
            return this.k1 < other.k1;
        }
        if (Math.abs(this.k2 - other.k2) > EPSILON) {
            return this.k2 < other.k2;
        }
        return false;
    }

    greaterThan(other) {
        return other.lessThan(this);
    }

    lessThanOrEqual(other) {
        return !other.lessThan(this);
    }

    greaterThanOrEqual(other) {
        return !this.lessThan(other);
    }

    equals(other) {
        return Math.abs(this.k1 - other.k1) <= EPSILON && Math.abs(this.k2 - other.k2) <= EPSILON;
    }
}

class GeometryJS {
    static euclidean(a, b) {
        if (!a || !b) return 0.0;
        const dim = Math.min(a.length, b.length);
        let sum = 0.0;
        for (let i = 0; i < dim; i++) {
            const diff = a[i] - b[i];
            sum += diff * diff;
        }
        return Math.sqrt(sum);
    }

    static distanceToBad(stateEmb, badIds, stateMap) {
        if (!badIds || badIds.length === 0) return INF;
        let minD = INF;
        for (const badId of badIds) {
            const badState = stateMap.get(badId);
            if (badState) {
                const d = this.euclidean(stateEmb, badState.embedding);
                if (d < minD) minD = d;
            }
        }
        return minD;
    }

    static pathSafetyScore(path, badIds, stateMap) {
        if (!path || path.length === 0 || !badIds || badIds.length === 0) return INF;
        let minSafety = INF;
        for (const id of path) {
            const s = stateMap.get(id);
            if (s) {
                const d = this.distanceToBad(s.embedding, badIds, stateMap);
                if (d < minSafety) minSafety = d;
            }
        }
        return minSafety;
    }
}

class LPAStarPlannerJS {
    constructor() {
        this.problem = null;
        this.statesMap = new Map();
        this.transitionsMap = new Map();
        this.badStateSet = new Set();
        this.successors = new Map();
        this.predecessors = new Map();
        this.g = new Map();
        this.rhs = new Map();
        this.queue = []; // Array of { key: Key, id: uint64 }
        this.minCostRatio = 1.0;
        this.exploredCount = 0;
        this.stepHistory = [];
        this.safetyMargin = 0.0;
        this.safetyWeight = 0.0;
        this.reliabilityWeight = 0.0;
    }

    loadProblem(problem) {
        this.problem = JSON.parse(JSON.stringify(problem));
        this.statesMap.clear();
        this.transitionsMap.clear();
        this.badStateSet.clear();
        this.successors.clear();
        this.predecessors.clear();
        this.g.clear();
        this.rhs.clear();
        this.queue = [];
        this.exploredCount = 0;
        this.stepHistory = [];

        for (const s of this.problem.states) {
            this.statesMap.set(s.id, s);
            this.g.set(s.id, INF);
            this.rhs.set(s.id, INF);
            this.successors.set(s.id, []);
            this.predecessors.set(s.id, []);
        }

        for (const badId of (this.problem.badStates || [])) {
            this.badStateSet.add(badId);
        }

        for (const t of this.problem.transitions) {
            this.transitionsMap.set(t.id, t);
            if (!this.successors.has(t.from)) this.successors.set(t.from, []);
            if (!this.predecessors.has(t.to)) this.predecessors.set(t.to, []);
            this.successors.get(t.from).push({ id: t.id, target: t.to, cost: t.cost, safety: t.safety, reliability: t.reliability, available: t.available !== false });
            this.predecessors.get(t.to).push({ id: t.id, target: t.from, cost: t.cost, safety: t.safety, reliability: t.reliability, available: t.available !== false });
        }

        this.calibrateHeuristic();

        if (!this.badStateSet.has(this.problem.initialState)) {
            this.rhs.set(this.problem.initialState, 0.0);
            const startKey = this.calculateKey(this.problem.initialState);
            this.insertQueue(this.problem.initialState, startKey);
        }
    }

    calibrateHeuristic() {
        let minRatio = INF;
        for (const t of this.problem.transitions) {
            if (t.available === false || t.cost <= 0) continue;
            const fromState = this.statesMap.get(t.from);
            const toState = this.statesMap.get(t.to);
            if (fromState && toState) {
                const dist = GeometryJS.euclidean(fromState.embedding, toState.embedding);
                if (dist > EPSILON) {
                    const ratio = t.cost / dist;
                    if (ratio < minRatio) minRatio = ratio;
                }
            }
        }
        this.minCostRatio = (minRatio < INF && minRatio > 0) ? minRatio : 1.0;
    }

    heuristic(u, goal) {
        if (u === goal) return 0.0;
        const sU = this.statesMap.get(u);
        const sG = this.statesMap.get(goal);
        if (!sU || !sG) return 0.0;
        return this.minCostRatio * GeometryJS.euclidean(sU.embedding, sG.embedding);
    }

    getG(u) {
        return this.g.has(u) ? this.g.get(u) : INF;
    }

    getRhs(u) {
        return this.rhs.has(u) ? this.rhs.get(u) : INF;
    }

    calculateKey(u) {
        const minVal = Math.min(this.getG(u), this.getRhs(u));
        if (minVal >= INF) return new Key(INF, INF);
        const hVal = this.heuristic(u, this.problem.goalState);
        return new Key(minVal + hVal, minVal);
    }

    insertQueue(u, key) {
        this.removeFromQueue(u);
        this.queue.push({ id: u, key: key });
        this.sortQueue();
    }

    removeFromQueue(u) {
        this.queue = this.queue.filter(item => item.id !== u);
    }

    sortQueue() {
        this.queue.sort((a, b) => a.key.lessThan(b.key) ? -1 : (a.key.greaterThan(b.key) ? 1 : 0));
    }

    computeEffectiveCost(edge) {
        let cost = edge.cost;
        if (this.safetyWeight > 0 && this.badStateSet.size > 0) {
            const targetState = this.statesMap.get(edge.target);
            if (targetState) {
                const d = GeometryJS.distanceToBad(targetState.embedding, Array.from(this.badStateSet), this.statesMap);
                if (d < this.safetyMargin) {
                    cost += this.safetyWeight * (this.safetyMargin - d);
                }
            }
        }
        if (this.reliabilityWeight > 0 && edge.reliability !== undefined) {
            cost += this.reliabilityWeight * (1.0 - edge.reliability);
        }
        return cost;
    }

    updateVertex(u) {
        if (this.badStateSet.has(u)) {
            this.removeFromQueue(u);
            this.g.set(u, INF);
            this.rhs.set(u, INF);
            return;
        }

        if (u !== this.problem.initialState) {
            let minRhs = INF;
            const preds = this.predecessors.get(u) || [];
            for (const edge of preds) {
                if (!edge.available || this.badStateSet.has(edge.target)) continue;
                const gPred = this.getG(edge.target);
                if (gPred < INF) {
                    const cand = gPred + this.computeEffectiveCost(edge);
                    if (cand < minRhs) minRhs = cand;
                }
            }
            this.rhs.set(u, minRhs);
        }

        this.removeFromQueue(u);
        const gVal = this.getG(u);
        const rhsVal = this.getRhs(u);
        if (Math.abs(gVal - rhsVal) > EPSILON) {
            this.insertQueue(u, this.calculateKey(u));
        }
    }

    computeShortestPath() {
        while (this.queue.length > 0) {
            const top = this.queue[0];
            const topKey = top.key;
            const u = top.id;

            const goalKey = this.calculateKey(this.problem.goalState);
            const gGoal = this.getG(this.problem.goalState);
            const rhsGoal = this.getRhs(this.problem.goalState);

            if (topKey.greaterThanOrEqual(goalKey) && Math.abs(gGoal - rhsGoal) <= EPSILON) {
                break;
            }

            this.queue.shift();
            this.exploredCount++;

            const gVal = this.getG(u);
            const rhsVal = this.getRhs(u);

            if (gVal > rhsVal) {
                this.g.set(u, rhsVal);
                this.stepHistory.push({
                    step: this.exploredCount,
                    vertex: u,
                    type: "Overconsistent -> Relaxed",
                    g: rhsVal,
                    rhs: rhsVal,
                    key: topKey
                });
                const succs = this.successors.get(u) || [];
                for (const edge of succs) {
                    this.updateVertex(edge.target);
                }
            } else {
                this.g.set(u, INF);
                this.stepHistory.push({
                    step: this.exploredCount,
                    vertex: u,
                    type: "Underconsistent -> Reset",
                    g: INF,
                    rhs: rhsVal,
                    key: topKey
                });
                this.updateVertex(u);
                const succs = this.successors.get(u) || [];
                for (const edge of succs) {
                    this.updateVertex(edge.target);
                }
            }
        }
    }

    // Step-by-step single execution step for visualization
    stepSingle() {
        if (this.queue.length === 0) return { done: true };
        const top = this.queue[0];
        const topKey = top.key;
        const u = top.id;

        const goalKey = this.calculateKey(this.problem.goalState);
        const gGoal = this.getG(this.problem.goalState);
        const rhsGoal = this.getRhs(this.problem.goalState);

        if (topKey.greaterThanOrEqual(goalKey) && Math.abs(gGoal - rhsGoal) <= EPSILON) {
            return { done: true };
        }

        this.queue.shift();
        this.exploredCount++;

        const gVal = this.getG(u);
        const rhsVal = this.getRhs(u);
        let eventType = "";

        if (gVal > rhsVal) {
            this.g.set(u, rhsVal);
            eventType = "Overconsistent (g > rhs) -> g updated to " + rhsVal.toFixed(2);
            const succs = this.successors.get(u) || [];
            for (const edge of succs) {
                this.updateVertex(edge.target);
            }
        } else {
            this.g.set(u, INF);
            eventType = "Underconsistent (g < rhs) -> g set to INF";
            this.updateVertex(u);
            const succs = this.successors.get(u) || [];
            for (const edge of succs) {
                this.updateVertex(edge.target);
            }
        }

        return {
            done: false,
            expandedVertex: u,
            eventType: eventType,
            key: topKey,
            queueSize: this.queue.length
        };
    }

    updateEdgeAvailability(transitionId, available) {
        const t = this.transitionsMap.get(transitionId);
        if (!t) return;
        t.available = available;

        const succs = this.successors.get(t.from) || [];
        for (const edge of succs) {
            if (edge.id === transitionId) edge.available = available;
        }
        const preds = this.predecessors.get(t.to) || [];
        for (const edge of preds) {
            if (edge.id === transitionId) edge.available = available;
        }

        this.updateVertex(t.to);
    }

    updateGoal(newGoal) {
        this.problem.goalState = newGoal;
        const activeIds = this.queue.map(item => item.id);
        this.queue = [];
        for (const u of activeIds) {
            this.insertQueue(u, this.calculateKey(u));
        }
        this.updateVertex(newGoal);
    }

    setBadState(stateId, isBad) {
        if (isBad) {
            this.badStateSet.add(stateId);
            this.removeFromQueue(stateId);
            this.g.set(stateId, INF);
            this.rhs.set(stateId, INF);
            const succs = this.successors.get(stateId) || [];
            for (const s of succs) this.updateVertex(s.target);
        } else {
            this.badStateSet.delete(stateId);
            this.updateVertex(stateId);
        }
    }

    addTransition(t) {
        this.problem.transitions.push(t);
        this.transitionsMap.set(t.id, t);
        if (!this.successors.has(t.from)) this.successors.set(t.from, []);
        if (!this.predecessors.has(t.to)) this.predecessors.set(t.to, []);
        this.successors.get(t.from).push({ id: t.id, target: t.to, cost: t.cost, safety: t.safety, reliability: t.reliability, available: t.available !== false });
        this.predecessors.get(t.to).push({ id: t.id, target: t.from, cost: t.cost, safety: t.safety, reliability: t.reliability, available: t.available !== false });
        this.updateVertex(t.to);
    }

    extractResult() {
        const result = {
            success: false,
            statePath: [],
            transitionPath: [],
            totalCost: 0.0,
            safetyScore: INF,
            cumulativeReliability: 1.0,
            exploredStates: this.exploredCount,
            objectiveScore: 0.0
        };

        const gGoal = this.getG(this.problem.goalState);
        if (gGoal >= INF || this.badStateSet.has(this.problem.goalState) || this.badStateSet.has(this.problem.initialState)) {
            return result;
        }

        let curr = this.problem.goalState;
        const revStates = [curr];
        const revTransitions = [];
        const visited = new Set([curr]);

        while (curr !== this.problem.initialState) {
            let bestPred = null;
            let bestTransId = null;
            let bestCostSum = INF;
            let chosenCost = 0.0;
            let chosenRel = 1.0;

            const preds = this.predecessors.get(curr) || [];
            for (const edge of preds) {
                if (!edge.available || this.badStateSet.has(edge.target)) continue;
                const gPred = this.getG(edge.target);
                if (gPred < INF) {
                    const cand = gPred + this.computeEffectiveCost(edge);
                    if (cand < bestCostSum) {
                        bestCostSum = cand;
                        bestPred = edge.target;
                        bestTransId = edge.id;
                        chosenCost = edge.cost;
                        chosenRel = edge.reliability !== undefined ? edge.reliability : 1.0;
                    }
                }
            }

            if (bestPred === null || visited.has(bestPred)) break;

            revTransitions.push(bestTransId);
            revStates.push(bestPred);
            visited.add(bestPred);
            result.totalCost += chosenCost;
            result.cumulativeReliability *= chosenRel;
            curr = bestPred;
        }

        if (curr !== this.problem.initialState) {
            result.success = false;
            return result;
        }

        result.success = true;
        result.statePath = revStates.reverse();
        result.transitionPath = revTransitions.reverse();
        result.safetyScore = GeometryJS.pathSafetyScore(result.statePath, Array.from(this.badStateSet), this.statesMap);

        const alpha = this.problem.alpha !== undefined ? this.problem.alpha : 100.0;
        const beta = this.problem.beta !== undefined ? this.problem.beta : 1.0;
        const gamma = this.problem.gamma !== undefined ? this.problem.gamma : 2.0;
        const delta = this.problem.delta !== undefined ? this.problem.delta : 5.0;

        const G = result.success ? 1.0 : 0.0;
        const C = result.totalCost;
        const D = (result.safetyScore < INF) ? result.safetyScore : 10.0;
        const R = result.cumulativeReliability;
        result.objectiveScore = (alpha * G) - (beta * C) + (gamma * D) + (delta * R);

        return result;
    }

    plan(problem) {
        const t0 = performance.now();
        this.loadProblem(problem);
        this.computeShortestPath();
        const t1 = performance.now();
        const res = this.extractResult();
        res.planningTimeMicroseconds = (t1 - t0) * 1000.0;
        return res;
    }

    replanIncremental() {
        const t0 = performance.now();
        this.exploredCount = 0;
        this.computeShortestPath();
        const t1 = performance.now();
        const res = this.extractResult();
        res.planningTimeMicroseconds = (t1 - t0) * 1000.0;
        return res;
    }
}
