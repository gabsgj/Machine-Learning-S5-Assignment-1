/**
 * planner-bridge.js — WASM ↔ JavaScript adapter layer.
 * 
 * Loads the Emscripten-compiled planner module and provides
 * a clean async API that converts between JS objects and C++ types.
 * All BigInt ↔ Number conversions happen here.
 */
import createPlannerModule from './bin/planner.js';

let Module = null;
let plannerInstance = null;

/**
 * Initialize the WASM module. Returns a bridge object with planner methods.
 */
export async function initPlanner() {
  if (Module) return createBridge();

  Module = await createPlannerModule({
    locateFile: (path) => {
      if (path.endsWith('.wasm')) {
        // import.meta.env.BASE_URL is '/' locally and '/Safe-Semantic-Planner/' on GH Pages.
        // Ensures planner.wasm is fetched from the correct subpath after deployment.
        return import.meta.env.BASE_URL + 'planner.wasm';
      }
      return path;
    },
  });
  return createBridge();
}

/**
 * Convert a JS number to BigInt for uint64_t fields
 */
function toBigInt(n) {
  return BigInt(n);
}

/**
 * Convert a BigInt back to Number (safe for small IDs)
 */
function toNumber(b) {
  return Number(b);
}

/**
 * Convert a JS test-case problem object into C++ PlanningProblem
 */
function toCppProblem(jsObj) {
  const problem = new Module.PlanningProblem();
  problem.initialState = toBigInt(jsObj.initialState);
  problem.goalState = toBigInt(jsObj.goalState);

  // Set bad states
  const badStates = new Module.VectorUint64();
  for (const bs of (jsObj.badStates || [])) {
    badStates.push_back(toBigInt(bs));
  }
  problem.badStates = badStates;

  // Set states
  const states = new Module.VectorState();
  for (const s of jsObj.states) {
    const state = new Module.State();
    state.id = toBigInt(s.id);
    const emb = new Module.VectorDouble();
    for (const v of s.embedding) {
      emb.push_back(v);
    }
    state.embedding = emb;
    states.push_back(state);
  }
  problem.states = states;

  // Set transitions
  const transitions = new Module.VectorTransition();
  for (const t of jsObj.transitions) {
    const tr = new Module.Transition();
    tr.id = toBigInt(t.id);
    tr.from = toBigInt(t.from);
    tr.to = toBigInt(t.to);
    tr.cost = t.cost;
    tr.safety = t.safety;
    tr.reliability = t.reliability;
    tr.available = t.available;
    transitions.push_back(tr);
  }
  problem.transitions = transitions;

  return problem;
}

/**
 * Convert a JS transition object into C++ Transition
 */
function toCppTransition(jsT) {
  const tr = new Module.Transition();
  tr.id = toBigInt(jsT.id);
  tr.from = toBigInt(jsT.from);
  tr.to = toBigInt(jsT.to);
  tr.cost = jsT.cost;
  tr.safety = jsT.safety;
  tr.reliability = jsT.reliability;
  tr.available = jsT.available;
  return tr;
}

/**
 * Convert a C++ PlanningResult back to a plain JS object
 */
function fromCppResult(cppResult) {
  const statePath = [];
  for (let i = 0; i < cppResult.statePath.size(); i++) {
    statePath.push(toNumber(cppResult.statePath.get(i)));
  }

  const transitionPath = [];
  for (let i = 0; i < cppResult.transitionPath.size(); i++) {
    transitionPath.push(toNumber(cppResult.transitionPath.get(i)));
  }

  return {
    success: cppResult.success,
    statePath,
    transitionPath,
    totalCost: cppResult.totalCost,
    safetyScore: cppResult.safetyScore,
  };
}

function createBridge() {
  return {
    /**
     * Run the full LPA* planner on a problem. Creates a new planner instance.
     */
    plan(jsProblem) {
      const cppProblem = toCppProblem(jsProblem);
      plannerInstance = new Module.LPAStarPlanner();
      const t0 = performance.now();
      const cppResult = plannerInstance.plan(cppProblem);
      const elapsed = performance.now() - t0;
      const result = fromCppResult(cppResult);
      return {
        ...result,
        statesExpanded: plannerInstance.statesExpanded,
        timeMs: elapsed,
      };
    },

    /**
     * Replan after disabling a transition.
     */
    onTransitionUnavailable(transitionId, fromState, toState) {
      if (!plannerInstance) throw new Error('Call plan() first');
      const t0 = performance.now();
      const cppResult = plannerInstance.onTransitionUnavailable(
        toBigInt(transitionId), toBigInt(fromState), toBigInt(toState)
      );
      const elapsed = performance.now() - t0;
      const result = fromCppResult(cppResult);
      return {
        ...result,
        statesExpanded: plannerInstance.statesExpanded,
        timeMs: elapsed,
      };
    },

    /**
     * Replan after adding a new transition.
     */
    onTransitionAdded(jsTransition) {
      if (!plannerInstance) throw new Error('Call plan() first');
      const cppT = toCppTransition(jsTransition);
      const t0 = performance.now();
      const cppResult = plannerInstance.onTransitionAdded(cppT);
      const elapsed = performance.now() - t0;
      const result = fromCppResult(cppResult);
      return {
        ...result,
        statesExpanded: plannerInstance.statesExpanded,
        timeMs: elapsed,
      };
    },

    /**
     * Replan after bad states change.
     */
    onBadStatesChanged(badStateIds) {
      if (!plannerInstance) throw new Error('Call plan() first');
      const vec = new Module.VectorUint64();
      for (const id of badStateIds) {
        vec.push_back(toBigInt(id));
      }
      const t0 = performance.now();
      const cppResult = plannerInstance.onBadStatesChanged(vec);
      const elapsed = performance.now() - t0;
      const result = fromCppResult(cppResult);
      return {
        ...result,
        statesExpanded: plannerInstance.statesExpanded,
        timeMs: elapsed,
      };
    },

    /**
     * Replan after changing the goal state.
     */
    onGoalChanged(newGoalId) {
      if (!plannerInstance) throw new Error('Call plan() first');
      const t0 = performance.now();
      const cppResult = plannerInstance.onGoalChanged(toBigInt(newGoalId));
      const elapsed = performance.now() - t0;
      const result = fromCppResult(cppResult);
      return {
        ...result,
        statesExpanded: plannerInstance.statesExpanded,
        timeMs: elapsed,
      };
    },

    /**
     * Check if the planner is ready (module loaded).
     */
    isReady() {
      return Module !== null;
    },

    /**
     * Check if a planner instance exists (plan() has been called).
     */
    hasInstance() {
      return plannerInstance !== null;
    },
  };
}
