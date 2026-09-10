/* Stateful WASM / JS Planner Host Web Worker.
 * All D* Lite solves and incremental updates execute in this background thread
 * so the frontend tab never freezes during complex or pathological searches.
 */

let wasmModule = null;
let wasmModulePromise = null;
let wasmPlannerHandle = 0;
let hasWasm = false;

// ── Fallback JS Planner inside Web Worker ──────────────────────────────────
class WorkerDStarLitePlanner {
  constructor() {
    this.attemptCount = 0;
    this.goalSuccessCount = 0;
    this.reset();
  }

  reset() {
    this.states = new Map();
    this.transitions = new Map();
    this.successors = new Map();
    this.predecessors = new Map();
    this.edgeLookup = new Map();
    this.g = new Map();
    this.rhs = new Map();
    this.inOpenList = new Map();
    this.openList = [];
    this.km = 0.0;
    this.cMin = 1.0;
    this.currentCallExpansions = 0;
    this.isInitialSolve = true;
    this.lastPlanningTimeMs = 0.0;
    this.lastReplanningTimeMs = 0.0;
    this.sInit = '';
    this.sGoal = '';
    this.sLastGoal = '';
    this.badStates = [];
    this.weights = { alpha: 1.0, beta: 1.0, gamma: 1.0, delta: 0.1, r: 0.0 };
    this.problem = null;
  }

  dist(a, b) {
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    return Math.sqrt(dx * dx + dy * dy);
  }

  computeHeuristic(u, target) {
    if (!this.states.has(u) || !this.states.has(target)) return 0.0;
    const d = this.dist(this.states.get(u).embedding, this.states.get(target).embedding);
    return this.cMin * d;
  }

  calculateKey(u) {
    const gVal = this.g.get(u) ?? Infinity;
    const rhsVal = this.rhs.get(u) ?? Infinity;
    const m = Math.min(gVal, rhsVal);
    if (m >= 1e9) return { k1: Infinity, k2: Infinity };
    return {
      k1: m + this.computeHeuristic(u, this.sGoal) + this.km,
      k2: m
    };
  }

  keyLess(a, b) {
    if (!a || !b) return false;
    const sameK1 = (a.k1 === b.k1) || Math.abs(a.k1 - b.k1) < 1e-9;
    if (!sameK1) return a.k1 < b.k1;
    const sameK2 = (a.k2 === b.k2) || Math.abs(a.k2 - b.k2) < 1e-9;
    return !sameK2 && a.k2 < b.k2;
  }

  initialize(problem, weights) {
    this.problem = JSON.parse(JSON.stringify(problem));
    this.weights = { ...weights };
    this.sInit = problem.initialState;
    this.sGoal = problem.goalState;
    this.sLastGoal = this.sGoal;
    this.km = 0.0;
    this.currentCallExpansions = 0;
    this.isInitialSolve = true;
    this.lastPlanningTimeMs = 0.0;
    this.lastReplanningTimeMs = 0.0;
    this.badStates = problem.badStates || [];

    this.states.clear();
    this.transitions.clear();
    this.successors.clear();
    this.predecessors.clear();
    this.edgeLookup.clear();
    this.g.clear();
    this.rhs.clear();
    this.inOpenList.clear();
    this.openList = [];

    const badSet = new Set(this.badStates);
    for (const s of problem.states) {
      const st = {
        id: s.id,
        label: s.label || s.id,
        embedding: s.embedding || [0, 0],
        isBadState: badSet.has(s.id),
        distanceToNearestBadState: Infinity,
        isExcluded: false
      };

      if (this.badStates.length > 0) {
        if (st.isBadState) {
          st.distanceToNearestBadState = 0.0;
        } else {
          let minDist = Infinity;
          for (const bId of this.badStates) {
            const bs = problem.states.find(x => x.id === bId);
            if (bs) {
              const d = this.dist(st.embedding, bs.embedding);
              if (d < minDist) minDist = d;
            }
          }
          st.distanceToNearestBadState = minDist;
        }
      }
      st.isExcluded = (st.distanceToNearestBadState <= this.weights.r);
      this.states.set(s.id, st);
      this.g.set(s.id, Infinity);
      this.rhs.set(s.id, Infinity);
      this.successors.set(s.id, []);
      this.predecessors.set(s.id, []);
    }

    let minCost = Infinity;
    let maxReliability = 0;
    this.cMin = Infinity;

    for (const t of problem.transitions) {
      const weight = this.weights.beta * t.cost - this.weights.delta * (t.reliability ?? 1.0);
      const edge = { ...t, weight, available: t.available !== false };
      this.transitions.set(t.id, edge);
      this.edgeLookup.set(`${t.from}->${t.to}`, t.id);
      this.successors.get(t.from)?.push(t.to);
      this.predecessors.get(t.to)?.push(t.from);

      const fromS = this.states.get(t.from);
      const toS = this.states.get(t.to);
      if (edge.available && fromS && toS && !fromS.isExcluded && !toS.isExcluded) {
        minCost = Math.min(minCost, t.cost);
        maxReliability = Math.max(maxReliability, t.reliability ?? 1.0);
        const d = this.dist(fromS.embedding, toS.embedding);
        if (d > 1e-9) {
          this.cMin = Math.min(this.cMin, weight / d);
        }
      }
    }

    if (this.cMin === Infinity) this.cMin = 1.0;

    const initS = this.states.get(this.sInit);
    if (initS && !initS.isExcluded) {
      this.rhs.set(this.sInit, 0.0);
      const key = this.calculateKey(this.sInit);
      this.inOpenList.set(this.sInit, key);
      this.openList.push({ stateId: this.sInit, key });
    }
  }

  updateVertex(u) {
    const st = this.states.get(u);
    if (!st) return;

    if (st.isExcluded) {
      this.rhs.set(u, Infinity);
      this.g.set(u, Infinity);
      this.inOpenList.delete(u);
      return;
    }

    if (u !== this.sInit) {
      let minRhs = Infinity;
      const preds = this.predecessors.get(u) || [];
      for (const p of preds) {
        const pState = this.states.get(p);
        if (pState && !pState.isExcluded) {
          const tId = this.edgeLookup.get(`${p}->${u}`);
          const edge = this.transitions.get(tId);
          if (edge && edge.available) {
            const gP = this.g.get(p) ?? Infinity;
            if (gP < 1e9) {
              const val = gP + edge.weight;
              if (val < minRhs) minRhs = val;
            }
          }
        }
      }
      this.rhs.set(u, minRhs);
    }

    const gVal = this.g.get(u) ?? Infinity;
    const rhsVal = this.rhs.get(u) ?? Infinity;
    const consistent = (gVal === rhsVal) || (Math.abs(gVal - rhsVal) < 1e-9);

    if (consistent) {
      this.inOpenList.delete(u);
    } else {
      const key = this.calculateKey(u);
      this.inOpenList.set(u, key);
      this.openList.push({ stateId: u, key });
    }
  }

  computeShortestPath() {
    const t0 = performance.now();
    this.currentCallExpansions = 0;
    this.attemptCount++;

    const initS = this.states.get(this.sInit);
    const goalS = this.states.get(this.sGoal);

    if (!initS || !goalS || initS.isExcluded || goalS.isExcluded) {
      const res = { success: false, errorMessage: "Initial or goal state is excluded by safety exclusion radius.", statePath: [], transitionPath: [], totalCost: 0, safetyScore: 0 };
      this.recordTiming(t0, false);
      return res;
    }

    while (true) {
      const openEmpty = this.openList.length === 0;
      let topKey = { k1: Infinity, k2: Infinity };
      let topEntry = null;
      let u = null;

      if (!openEmpty) {
        this.openList.sort((a, b) => this.keyLess(a.key, b.key) ? -1 : 1);
        topEntry = this.openList[0];
        topKey = topEntry.key;
        u = topEntry.stateId;
      }

      const goalKey = this.calculateKey(this.sGoal);
      const gGoal = this.g.get(this.sGoal) ?? Infinity;
      const rhsGoal = this.rhs.get(this.sGoal) ?? Infinity;

      const goalConsistent = (gGoal === rhsGoal) || (Math.abs(gGoal - rhsGoal) < 1e-9);
      const topLess = this.keyLess(topKey, goalKey);

      if (!topLess && goalConsistent) break;
      if (openEmpty) break; // Priority queue exhausted, goal unreachable

      this.openList.shift();

      const storedKey = this.inOpenList.get(u);
      if (!storedKey) continue;
      if (Math.abs(storedKey.k1 - topKey.k1) > 1e-9 || Math.abs(storedKey.k2 - topKey.k2) > 1e-9) continue;

      const curKey = this.calculateKey(u);
      const gU = this.g.get(u) ?? Infinity;
      const rhsU = this.rhs.get(u) ?? Infinity;

      if (this.keyLess(topKey, curKey)) {
        this.inOpenList.set(u, curKey);
        this.openList.push({ stateId: u, key: curKey });
      } else if (gU > rhsU) {
        this.inOpenList.delete(u);
        this.currentCallExpansions++;
        this.g.set(u, rhsU);
        const succs = this.successors.get(u) || [];
        for (const s of succs) this.updateVertex(s);
      } else {
        this.currentCallExpansions++;
        this.g.set(u, Infinity);
        this.updateVertex(u);
        const succs = this.successors.get(u) || [];
        for (const s of succs) this.updateVertex(s);
      }
    }

    const res = this.extractPath();
    if (res.success) this.goalSuccessCount++;
    this.recordTiming(t0, res.success);
    return res;
  }

  recordTiming(t0, success) {
    const elapsed = performance.now() - t0;
    if (this.isInitialSolve) {
      this.lastPlanningTimeMs = elapsed;
      this.isInitialSolve = false;
    } else {
      this.lastReplanningTimeMs = elapsed;
    }
  }

  extractPath() {
    const gGoal = this.g.get(this.sGoal) ?? Infinity;
    if (gGoal >= 1e9) {
      return { success: false, errorMessage: "No valid path found to destination goal.", statePath: [], transitionPath: [], totalCost: 0, safetyScore: 0 };
    }

    const pathStates = [this.sGoal];
    const pathTransitions = [];
    let curr = this.sGoal;
    const visited = new Set([curr]);
    let totalCost = 0;
    let totalSafety = 0;

    while (curr !== this.sInit) {
      let bestPred = '';
      let bestTransId = '';
      let bestVal = Infinity;

      const preds = this.predecessors.get(curr) || [];
      for (const p of preds) {
        const pState = this.states.get(p);
        if (pState && !pState.isExcluded) {
          const tId = this.edgeLookup.get(`${p}->${curr}`);
          const edge = this.transitions.get(tId);
          if (edge && edge.available) {
            const gP = this.g.get(p) ?? Infinity;
            if (gP < 1e9) {
              const val = gP + edge.weight;
              if (val < bestVal) {
                bestVal = val;
                bestPred = p;
                bestTransId = tId;
              }
            }
          }
        }
      }

      if (!bestPred || visited.has(bestPred)) {
        return { success: false, errorMessage: "Disconnected predecessor or loop encountered.", statePath: [], transitionPath: [], totalCost: 0, safetyScore: 0 };
      }

      const edge = this.transitions.get(bestTransId);
      totalCost += edge.cost;
      totalSafety += (edge.safetyScore ?? 1.0);
      pathTransitions.push(bestTransId);
      curr = bestPred;
      pathStates.push(curr);
      visited.add(curr);
    }

    pathStates.reverse();
    pathTransitions.reverse();

    return {
      success: true,
      statePath: pathStates,
      transitionPath: pathTransitions,
      totalCost: totalCost,
      safetyScore: pathTransitions.length === 0 ? 1.0 : (totalSafety / pathTransitions.length),
      errorMessage: ""
    };
  }

  notifyEdgeChanged(transitionId, newCost, newAvailability) {
    const edge = this.transitions.get(transitionId);
    if (!edge) return;
    edge.cost = newCost;
    edge.available = newAvailability;
    edge.weight = this.weights.beta * newCost - this.weights.delta * (edge.reliability ?? 1.0);
    this.updateVertex(edge.to);
  }

  notifyGoalChanged(newGoalStateId) {
    if (newGoalStateId === this.sGoal) return;
    this.km += this.computeHeuristic(this.sLastGoal, newGoalStateId);
    this.sGoal = newGoalStateId;
    this.sLastGoal = this.sGoal;
    if (this.states.has(this.sGoal)) {
      this.updateVertex(this.sGoal);
    }
  }

  notifyBadStatesChanged(newBadStates) {
    this.badStates = [...newBadStates];
    this.problem.badStates = [...newBadStates];
    const badSet = new Set(this.badStates);
    for (const s of this.states.values()) {
      s.isBadState = badSet.has(s.id);
      if (this.badStates.length === 0) s.distanceToNearestBadState = Infinity;
      else if (s.isBadState) s.distanceToNearestBadState = 0.0;
      else {
        let minDist = Infinity;
        for (const badId of this.badStates) {
          const bad = this.states.get(badId);
          if (bad) minDist = Math.min(minDist, this.dist(s.embedding, bad.embedding));
        }
        s.distanceToNearestBadState = minDist;
      }
    }
    this.updateSafetyExclusions();
  }

  updateSafetyExclusions() {
    const changed = [];
    for (const [id, s] of this.states) {
      const prev = s.isExcluded;
      s.isExcluded = (s.distanceToNearestBadState <= this.weights.r);
      if (prev !== s.isExcluded) changed.push(id);
    }
    for (const sId of changed) {
      this.updateVertex(sId);
      const succs = this.successors.get(sId) || [];
      for (const succ of succs) this.updateVertex(succ);
    }
  }

  getMetrics() {
    let activeCount = 0;
    for (const [_, s] of this.states) {
      if (!s.isExcluded) activeCount++;
    }

    const sizeofGRhs = 48;
    const sizeofHeap = 40;
    const sizeofKdNode = 72;
    const memBytes = (sizeofGRhs * activeCount) + (sizeofHeap * this.openList.length) + (sizeofKdNode * this.badStates.length);

    const res = this.extractPath();
    let badVisited = 0;
    let minClear = Infinity;

    if (res.success && res.statePath.length > 0) {
      for (const sId of res.statePath) {
        const s = this.states.get(sId);
        if (s) {
          if (s.isBadState) badVisited++;
          if (s.distanceToNearestBadState < minClear) minClear = s.distanceToNearestBadState;
        }
      }
    }

    return {
      statesExplored: this.currentCallExpansions,
      planningTimeMs: this.lastPlanningTimeMs,
      replanningTimeMs: this.lastReplanningTimeMs,
      memoryBytes: memBytes,
      totalCost: res.success ? res.totalCost : 0.0,
      minClearance: minClear === Infinity ? 0.0 : minClear,
      badStatesVisited: badVisited,
      goalSuccessCount: this.goalSuccessCount,
      attemptCount: this.attemptCount,
      goalSuccessRate: this.attemptCount > 0 ? (this.goalSuccessCount / this.attemptCount) : 0.0
    };
  }
}

const workerJsPlanner = new WorkerDStarLitePlanner();

// ── WASM Loader ────────────────────────────────────────────────────────────
async function initWasmIfAvailable() {
  if (wasmModulePromise) return wasmModulePromise;
  wasmModulePromise = (async () => {
    try {
      if (typeof importScripts === 'function') {
        importScripts('wasm/planner.js');
        if (typeof self.PlannerModule === 'function') {
          wasmModule = await self.PlannerModule();
          hasWasm = true;
        }
      }
    } catch (_) {
      hasWasm = false;
    }
    return wasmModule;
  })();
  return wasmModulePromise;
}

// Kick off optional WASM discovery
initWasmIfAvailable().catch(() => {});

// ── Web Worker Message Dispatcher ──────────────────────────────────────────
self.onmessage = async ({ data }) => {
  const { id, type, payload = {} } = data;
  try {
    await initWasmIfAvailable();

    if (hasWasm && wasmModule) {
      // WASM-accelerated path
      if (type === 'initialize') {
        if (wasmPlannerHandle) wasmModule.cwrap('destroy_planner', null, ['number'])(wasmPlannerHandle);
        wasmPlannerHandle = wasmModule.cwrap('create_planner', 'number', ['string', 'string'])(
          JSON.stringify(payload.problem), JSON.stringify(payload.weights)
        );
        if (wasmPlannerHandle < 1) throw new Error('WASM planner initialization failed');
        const rawRes = JSON.parse(wasmModule.cwrap('get_result', 'string', ['number'])(wasmPlannerHandle));
        const rawMet = JSON.parse(wasmModule.cwrap('get_metrics', 'string', ['number'])(wasmPlannerHandle));
        self.postMessage({ id, ok: true, engine: 'wasm', result: rawRes.result, metrics: rawMet });
        return;
      }
      if (!wasmPlannerHandle) throw new Error('WASM planner is not initialized');
      const api = {
        edge: ['notify_edge_changed', { transitionId: payload.transitionId, newCost: payload.newCost, newAvailability: payload.newAvailability }],
        goal: ['notify_goal_changed', { newGoalStateId: payload.newGoalStateId }],
        badStates: ['notify_bad_states_changed', { badStates: payload.badStates }],
        weights: ['recompute_with_weights', payload.weights],
        recompute: ['get_result', null]
      }[type];

      if (!api) throw new Error(`Unknown worker command: ${type}`);
      let reply;
      if (type === 'recompute') {
        const rawRes = JSON.parse(wasmModule.cwrap('get_result', 'string', ['number'])(wasmPlannerHandle));
        const rawMet = JSON.parse(wasmModule.cwrap('get_metrics', 'string', ['number'])(wasmPlannerHandle));
        reply = { result: rawRes.result, metrics: rawMet };
      } else {
        const raw = wasmModule.cwrap(api[0], 'string', ['number', 'string'])(wasmPlannerHandle, JSON.stringify(api[1]));
        reply = JSON.parse(raw);
      }
      if (reply.error) throw new Error(reply.error);
      self.postMessage({ id, ok: true, engine: 'wasm', result: reply.result, metrics: reply.metrics });
      return;
    }

    // Worker JS Planner path
    let result, metrics;
    if (type === 'initialize') {
      workerJsPlanner.initialize(payload.problem, payload.weights);
      result = workerJsPlanner.computeShortestPath();
      metrics = workerJsPlanner.getMetrics();
    } else if (type === 'edge') {
      workerJsPlanner.notifyEdgeChanged(payload.transitionId, payload.newCost, payload.newAvailability);
      result = workerJsPlanner.computeShortestPath();
      metrics = workerJsPlanner.getMetrics();
    } else if (type === 'goal') {
      workerJsPlanner.notifyGoalChanged(payload.newGoalStateId);
      result = workerJsPlanner.computeShortestPath();
      metrics = workerJsPlanner.getMetrics();
    } else if (type === 'badStates') {
      workerJsPlanner.notifyBadStatesChanged(payload.badStates);
      result = workerJsPlanner.computeShortestPath();
      metrics = workerJsPlanner.getMetrics();
    } else if (type === 'weights') {
      workerJsPlanner.weights = payload.weights;
      workerJsPlanner.notifySafetyRadiusChanged(payload.weights.r);
      result = workerJsPlanner.computeShortestPath();
      metrics = workerJsPlanner.getMetrics();
    } else if (type === 'recompute') {
      result = workerJsPlanner.computeShortestPath();
      metrics = workerJsPlanner.getMetrics();
    } else {
      throw new Error(`Unknown worker command: ${type}`);
    }

    self.postMessage({ id, ok: true, engine: 'worker_js', result, metrics });
  } catch (error) {
    self.postMessage({ id, ok: false, error: error.message || String(error) });
  }
};
