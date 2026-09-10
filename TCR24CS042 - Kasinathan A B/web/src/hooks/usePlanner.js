/**
 * usePlanner — React hook that manages the WASM planner lifecycle.
 * Handles initialization, planning, replanning, and state tracking.
 */
import { useState, useCallback, useRef, useEffect } from 'react';
import { initPlanner } from '../wasm/planner-bridge.js';

export function usePlanner() {
  const [wasmStatus, setWasmStatus] = useState('loading'); // 'loading' | 'ready' | 'error'
  const [result, setResult] = useState(null);
  const [history, setHistory] = useState([]); // array of { label, result }
  const [isRunning, setIsRunning] = useState(false);
  const bridgeRef = useRef(null);

  // Initialize WASM module on mount
  useEffect(() => {
    let cancelled = false;
    initPlanner()
      .then(bridge => {
        if (!cancelled) {
          bridgeRef.current = bridge;
          setWasmStatus('ready');
        }
      })
      .catch(err => {
        console.error('WASM init failed:', err);
        if (!cancelled) setWasmStatus('error');
      });
    return () => { cancelled = true; };
  }, []);

  const plan = useCallback((problem, label = 'Initial plan') => {
    if (!bridgeRef.current) return null;
    setIsRunning(true);
    try {
      const r = bridgeRef.current.plan(problem);
      setResult(r);
      setHistory([{ label, result: r }]);
      return r;
    } catch (err) {
      console.error('Plan failed:', err);
      return null;
    } finally {
      setIsRunning(false);
    }
  }, []);

  const replan = useCallback((action, label = 'Replan') => {
    if (!bridgeRef.current || !bridgeRef.current.hasInstance()) return null;
    setIsRunning(true);
    try {
      let r;
      switch (action.type) {
        case 'disableTransition':
          r = bridgeRef.current.onTransitionUnavailable(
            action.transitionId, action.fromState, action.toState
          );
          break;
        case 'addTransition':
          r = bridgeRef.current.onTransitionAdded(action.transition);
          break;
        case 'changeBadStates':
          r = bridgeRef.current.onBadStatesChanged(action.badStates);
          break;
        case 'changeGoal':
          r = bridgeRef.current.onGoalChanged(action.newGoal);
          break;
        default:
          throw new Error(`Unknown replan action: ${action.type}`);
      }
      setResult(r);
      setHistory(prev => [...prev, { label, result: r }]);
      return r;
    } catch (err) {
      console.error('Replan failed:', err);
      return null;
    } finally {
      setIsRunning(false);
    }
  }, []);

  const clearResults = useCallback(() => {
    setResult(null);
    setHistory([]);
  }, []);

  return {
    wasmStatus,
    result,
    history,
    isRunning,
    plan,
    replan,
    clearResults,
  };
}
