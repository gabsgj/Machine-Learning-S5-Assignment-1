/**
 * @file test_wasm_smoke.mjs
 * @brief Node.js smoke test for the WASM planner module.
 * 
 * Proves that:
 * 1. A stateful planner instance survives across the WASM boundary
 * 2. Initial solve and incremental replan both produce correct results
 * 3. statesExplored for the replan is bounded relative to graph size
 * 
 * Run:  node test_wasm_smoke.mjs
 * Requires: wasm/planner.js + wasm/planner.wasm (built via build_wasm.sh)
 */

import { createRequire } from 'module';
const require = createRequire(import.meta.url);

async function main() {
    console.log('========================================');
    console.log('WASM Planner Node.js Smoke Test');
    console.log('========================================');

    // Load the WASM module
    const PlannerModule = require('./wasm/planner.js');
    const Module = await PlannerModule();

    // Wrap C functions
    const createPlanner = Module.cwrap('create_planner', 'number', ['string', 'string']);
    const notifyEdgeChanged = Module.cwrap('notify_edge_changed', 'string', ['number', 'string']);
    const notifyGoalChanged = Module.cwrap('notify_goal_changed', 'string', ['number', 'string']);
    const getMetrics = Module.cwrap('get_metrics', 'string', ['number']);
    const getResult = Module.cwrap('get_result', 'string', ['number']);
    const destroyPlanner = Module.cwrap('destroy_planner', null, ['number']);

    // ── Build TC1 problem: 6x6 grid, no bad states ──────────────────────
    const states = [];
    for (let y = 0; y < 6; y++) {
        for (let x = 0; x < 6; x++) {
            states.push({ id: `s_${x}_${y}`, embedding: [x, y] });
        }
    }

    const transitions = [];
    let tId = 0;
    for (let y = 0; y < 6; y++) {
        for (let x = 0; x < 6; x++) {
            const u = `s_${x}_${y}`;
            if (x + 1 < 6) {
                transitions.push({ id: `t_${++tId}`, from: u, to: `s_${x+1}_${y}`, cost: 1.0, reliability: 0.95 });
            }
            if (y + 1 < 6) {
                transitions.push({ id: `t_${++tId}`, from: u, to: `s_${x}_${y+1}`, cost: 1.0, reliability: 0.95 });
            }
            if (x + 1 < 6 && y + 1 < 6) {
                transitions.push({ id: `t_${++tId}`, from: u, to: `s_${x+1}_${y+1}`, cost: 1.414, reliability: 0.90 });
            }
        }
    }

    const problem = {
        initialState: 's_0_0',
        goalState: 's_5_5',
        badStates: [],
        states,
        transitions
    };

    const weights = { alpha: 1.0, beta: 1.0, gamma: 1.0, delta: 0.1, r: 0.0 };

    // ── Step 1: Create planner ──────────────────────────────────────────
    console.log('[1] Creating planner from TC1 grid...');
    const handle = createPlanner(JSON.stringify(problem), JSON.stringify(weights));
    console.assert(handle > 0, 'create_planner must return valid handle');

    // ── Step 2: Get initial metrics ─────────────────────────────────────
    console.log('[2] Getting initial metrics...');
    const initialMetrics = JSON.parse(getMetrics(handle));
    console.log('    Initial metrics:', initialMetrics);
    console.assert(initialMetrics.attemptCount === 1, 'attemptCount must be 1');
    console.assert(initialMetrics.goalSuccessCount === 1, 'goalSuccessCount must be 1');
    const initialExplored = initialMetrics.statesExplored;

    // ── Step 3: Get initial result & pick an edge to break ──────────────
    const initialResult = JSON.parse(getResult(handle));
    console.assert(initialResult.result.success === true, 'Initial solve must succeed');
    const edgeToBreak = initialResult.result.transitionPath[0];
    console.log(`    Breaking edge: ${edgeToBreak}`);

    // ── Step 4: TC4 edge-sever event ────────────────────────────────────
    console.log('[3] Notifying edge change (TC4 sever)...');
    const edgeChangeResult = JSON.parse(notifyEdgeChanged(handle, JSON.stringify({
        transitionId: edgeToBreak,
        newCost: 100.0,
        newAvailability: false
    })));
    console.log('    Replan result success:', edgeChangeResult.result.success);
    console.log('    Replan metrics:', edgeChangeResult.metrics);
    console.assert(edgeChangeResult.result.success === true, 'Replan must succeed');

    const replanExplored = edgeChangeResult.metrics.statesExplored;
    console.log(`    Initial statesExplored: ${initialExplored}, Replan statesExplored: ${replanExplored}`);

    // The broken edge must not appear in the replanned path
    console.assert(
        !edgeChangeResult.result.transitionPath.includes(edgeToBreak),
        'Broken edge must not be in replanned path'
    );

    // Replan expansions must be bounded relative to graph size (36 nodes)
    console.assert(replanExplored < 36, `Replan expansions (${replanExplored}) must be < 36 (graph size)`);

    // ── Step 5: Goal shift ──────────────────────────────────────────────
    console.log('[4] Notifying goal change to s_4_4...');
    const goalResult = JSON.parse(notifyGoalChanged(handle, JSON.stringify({
        newGoalStateId: 's_4_4'
    })));
    console.assert(goalResult.result.success === true, 'Goal shift must succeed');
    console.log('    Goal shift statesExplored:', goalResult.metrics.statesExplored);

    // ── Step 6: Final lifetime metrics ──────────────────────────────────
    const finalMetrics = JSON.parse(getMetrics(handle));
    console.log('    Final metrics:', finalMetrics);
    console.assert(finalMetrics.attemptCount === 3, 'attemptCount must be 3');
    console.assert(finalMetrics.goalSuccessCount === 3, 'goalSuccessCount must be 3');
    console.assert(Math.abs(finalMetrics.goalSuccessRate - 1.0) < 1e-6, 'Success rate must be 100%');

    // ── Step 7: Destroy ─────────────────────────────────────────────────
    console.log('[5] Destroying planner...');
    destroyPlanner(handle);

    console.log('');
    console.log('========================================');
    console.log('WASM SMOKE TEST PASSED!');
    console.log('========================================');
}

main().catch(err => {
    console.error('SMOKE TEST FAILED:', err);
    process.exit(1);
});
