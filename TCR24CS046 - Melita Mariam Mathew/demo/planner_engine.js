/**
 * Safe Semantic Planner Engine (JavaScript 1:1 Implementation of C++ Core)
 * Implements LPA*, D* Lite, Multi-Objective Safety Repulsive Potential & Dynamic Replanning
 */

class LPAKey {
    constructor(k1 = Infinity, k2 = Infinity) {
        this.k1 = k1;
        this.k2 = k2;
    }

    lessThan(other) {
        if (Math.abs(this.k1 - other.k1) > 1e-9) {
            return this.k1 < other.k1;
        }
        return this.k2 < other.k2;
    }

    lessThanOrEqual(other) {
        return this.lessThan(other) || (Math.abs(this.k1 - other.k1) <= 1e-9 && Math.abs(this.k2 - other.k2) <= 1e-9);
    }
}

class JSPlannerEngine {
    constructor() {
        this.alpha = 100.0;
        this.beta = 1.0;
        this.gamma = 5.0;
        this.delta = 2.0;
        this.safetyRadius = 3.0;
        this.algoType = 'lpastar'; // 'lpastar' or 'dstarlite'

        this.states = new Map(); // id -> {id, x, y, embedding, name}
        this.transitions = new Map(); // id -> {id, from, to, cost, safety, reliability, available}
        this.badStates = new Set();
        this.initialState = 0;
        this.goalState = 3;

        this.g = new Map();
        this.rhs = new Map();
        this.openSet = new Map(); // id -> LPAKey
        this.km = 0.0;
        this.lastStart = 0;
        this.exploredCount = 0;
        this.executionTimeMs = 0;
    }

    setProblem(problem) {
        this.states.clear();
        this.transitions.clear();
        this.badStates.clear();
        this.initialState = problem.initialState;
        this.goalState = problem.goalState;

        for (const s of problem.states) {
            this.states.set(s.id, {
                id: s.id,
                x: s.embedding[0] || 0,
                y: s.embedding[1] || 0,
                embedding: s.embedding,
                name: s.name || `S${s.id}`
            });
        }

        for (const b of (problem.badStates || [])) {
            this.badStates.add(b);
        }

        for (const t of problem.transitions) {
            this.transitions.set(t.id, {
                id: t.id,
                from: t.from,
                to: t.to,
                cost: t.cost !== undefined ? t.cost : 1.0,
                safety: t.safety !== undefined ? t.safety : 1.0,
                reliability: t.reliability !== undefined ? t.reliability : 0.99,
                available: t.available !== undefined ? t.available : true
            });
        }

        this.initLPA();
    }

    initLPA() {
        this.g.clear();
        this.rhs.clear();
        this.openSet.clear();
        this.exploredCount = 0;

        for (const [id] of this.states) {
            this.g.set(id, Infinity);
            this.rhs.set(id, Infinity);
        }

        this.rhs.set(this.initialState, 0.0);
        const kStart = this.calculateKey(this.initialState);
        this.openSet.set(this.initialState, kStart);
    }

    heuristic(fromId, toId) {
        const sFrom = this.states.get(fromId);
        const sTo = this.states.get(toId);
        if (!sFrom || !sTo) return 0.0;
        const dx = sFrom.x - sTo.x;
        const dy = sFrom.y - sTo.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    minDistanceToBadStates(stateId) {
        const s = this.states.get(stateId);
        if (!s || this.badStates.size === 0) return 100.0;
        let minD = Infinity;
        for (const bId of this.badStates) {
            const b = this.states.get(bId);
            if (b) {
                const dx = s.x - b.x;
                const dy = s.y - b.y;
                const d = Math.sqrt(dx * dx + dy * dy);
                if (d < minD) minD = d;
            }
        }
        return minD;
    }

    effectiveTransitionCost(t) {
        if (!t.available) return Infinity;
        if (this.badStates.has(t.to) || this.badStates.has(t.from)) return Infinity;

        const baseC = Math.max(0.001, t.cost) * this.beta;
        const relPenalty = this.delta * (1.0 - Math.min(1.0, Math.max(0.0, t.reliability)));

        let safetyPenalty = 0.0;
        if (this.badStates.size > 0) {
            const minDist = this.minDistanceToBadStates(t.to);
            if (minDist < this.safetyRadius) {
                const margin = this.safetyRadius - minDist;
                safetyPenalty = this.gamma * (margin * margin + (1.0 / (minDist + 0.1)));
            }
        }

        return baseC + relPenalty + safetyPenalty;
    }

    calculateKey(stateId) {
        const gVal = this.g.get(stateId) !== undefined ? this.g.get(stateId) : Infinity;
        const rhsVal = this.rhs.get(stateId) !== undefined ? this.rhs.get(stateId) : Infinity;
        const m = Math.min(gVal, rhsVal);
        const hVal = this.heuristic(stateId, this.goalState);
        return new LPAKey(m + hVal, m);
    }

    getInTransitions(stateId) {
        const list = [];
        for (const [_, t] of this.transitions) {
            if (t.to === stateId) list.push(t);
        }
        return list;
    }

    getOutTransitions(stateId) {
        const list = [];
        for (const [_, t] of this.transitions) {
            if (t.from === stateId) list.push(t);
        }
        return list;
    }

    updateVertex(u) {
        if (u !== this.initialState) {
            let minRhs = Infinity;
            const inEdges = this.getInTransitions(u);
            for (const t of inEdges) {
                const gPred = this.g.get(t.from) !== undefined ? this.g.get(t.from) : Infinity;
                const cEff = this.effectiveTransitionCost(t);
                if (gPred < Infinity / 2 && cEff < Infinity / 2) {
                    const val = gPred + cEff;
                    if (val < minRhs) minRhs = val;
                }
            }
            this.rhs.set(u, minRhs);
        }

        this.openSet.delete(u);

        const gVal = this.g.get(u) !== undefined ? this.g.get(u) : Infinity;
        const rhsVal = this.rhs.get(u) !== undefined ? this.rhs.get(u) : Infinity;

        if (Math.abs(gVal - rhsVal) > 1e-9) {
            this.openSet.set(u, this.calculateKey(u));
        }
    }

    getTopOpenNode() {
        let bestId = null;
        let bestKey = null;

        for (const [id, key] of this.openSet) {
            if (!bestKey || key.lessThan(bestKey)) {
                bestKey = key;
                bestId = id;
            }
        }
        return { id: bestId, key: bestKey };
    }

    computeShortestPath() {
        const tStart = performance.now();
        let maxIterations = 2000;

        while (this.openSet.size > 0 && maxIterations-- > 0) {
            const top = this.getTopOpenNode();
            if (top.id === null) break;

            const keyGoal = this.calculateKey(this.goalState);
            const gGoal = this.g.get(this.goalState);
            const rhsGoal = this.rhs.get(this.goalState);

            if (!top.key.lessThan(keyGoal) && Math.abs(gGoal - rhsGoal) <= 1e-9) {
                break;
            }

            const u = top.id;
            this.openSet.delete(u);
            this.exploredCount++;

            const gVal = this.g.get(u);
            const rhsVal = this.rhs.get(u);

            if (gVal > rhsVal) {
                this.g.set(u, rhsVal);
                for (const t of this.getOutTransitions(u)) {
                    this.updateVertex(t.to);
                }
            } else {
                this.g.set(u, Infinity);
                this.updateVertex(u);
                for (const t of this.getOutTransitions(u)) {
                    this.updateVertex(t.to);
                }
            }
        }

        const tEnd = performance.now();
        this.executionTimeMs = (tEnd - tStart);

        return this.extractPath();
    }

    extractPath() {
        const gGoal = this.g.get(this.goalState);
        if (!gGoal || gGoal >= Infinity / 2) {
            return {
                success: false,
                statePath: [],
                transitionPath: [],
                totalCost: 0,
                minSafetyDistance: 0,
                cumulativeReliability: 0,
                badStatesVisited: 0,
                compositeScore: 0,
                exploredStates: this.exploredCount,
                planningTimeMs: this.executionTimeMs
            };
        }

        const pathStates = [this.goalState];
        const pathTransitions = [];
        let curr = this.goalState;
        const visited = new Set([curr]);

        while (curr !== this.initialState) {
            let bestPredVal = Infinity;
            let bestPred = curr;
            let bestTid = null;

            for (const t of this.getInTransitions(curr)) {
                const gPred = this.g.get(t.from);
                const cEff = this.effectiveTransitionCost(t);
                if (gPred < Infinity / 2 && cEff < Infinity / 2) {
                    const val = gPred + cEff;
                    if (val < bestPredVal) {
                        bestPredVal = val;
                        bestPred = t.from;
                        bestTid = t.id;
                    }
                }
            }

            if (bestPred === curr || visited.has(bestPred)) break;
            pathStates.push(bestPred);
            pathTransitions.push(bestTid);
            visited.add(bestPred);
            curr = bestPred;
        }

        if (curr !== this.initialState) {
            return { success: false, statePath: [], transitionPath: [] };
        }

        pathStates.reverse();
        pathTransitions.reverse();

        let totalRawCost = 0.0;
        let cumReliability = 1.0;
        let minSafeDist = Infinity;
        let badVisits = 0;

        for (const tid of pathTransitions) {
            const t = this.transitions.get(tid);
            if (t) {
                totalRawCost += t.cost;
                cumReliability *= Math.min(1.0, Math.max(0.0, t.reliability));
            }
        }

        for (const sid of pathStates) {
            if (this.badStates.has(sid)) badVisits++;
            const d = this.minDistanceToBadStates(sid);
            if (d < minSafeDist) minSafeDist = d;
        }

        if (minSafeDist >= Infinity / 2) minSafeDist = 100.0;

        const score = (this.alpha * 1.0) - (this.beta * totalRawCost) + (this.gamma * minSafeDist) + (this.delta * cumReliability);

        return {
            success: true,
            statePath: pathStates,
            transitionPath: pathTransitions,
            totalCost: totalRawCost,
            minSafetyDistance: minSafeDist,
            cumulativeReliability: cumReliability,
            badStatesVisited: badVisits,
            compositeScore: score,
            exploredStates: this.exploredCount,
            planningTimeMs: this.executionTimeMs
        };
    }

    // Dynamic Updates
    toggleTransitionAvailability(tid) {
        const t = this.transitions.get(tid);
        if (!t) return;
        t.available = !t.available;
        this.updateVertex(t.to);
        return this.computeShortestPath();
    }

    toggleBadState(stateId) {
        if (stateId === this.initialState || stateId === this.goalState) return;
        if (this.badStates.has(stateId)) {
            this.badStates.delete(stateId);
        } else {
            this.badStates.add(stateId);
        }
        for (const [id] of this.states) {
            this.updateVertex(id);
        }
        return this.computeShortestPath();
    }

    setGoal(newGoal) {
        this.goalState = newGoal;
        for (const [id] of this.openSet) {
            this.openSet.set(id, this.calculateKey(id));
        }
        return this.computeShortestPath();
    }

    addTransition(from, to, cost = 1.0, rel = 0.99) {
        const newId = Date.now();
        const t = { id: newId, from, to, cost, safety: 1.0, reliability: rel, available: true };
        this.transitions.set(newId, t);
        this.updateVertex(to);
        return this.computeShortestPath();
    }
}
