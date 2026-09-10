import { useState, useCallback, useEffect } from 'react';
import { usePlanner } from './hooks/usePlanner';
import { Sidebar } from './components/Sidebar';
import { GraphCanvas } from './components/GraphCanvas';
import { ResultsPanel } from './components/ResultsPanel';
import { ActionPanel } from './components/ActionPanel';
import { MetricsBar } from './components/MetricsBar';
import { DataTable } from './components/DataTable';
import { ThemeToggle } from './components/ThemeToggle';
import { testCases } from './data/test-cases';

export default function App() {
  const { wasmStatus, result, history, isRunning, plan, replan } = usePlanner();

  const [selectedTestCase, setSelectedTestCase] = useState(testCases[0]);
  const [selectedNode, setSelectedNode] = useState(null);
  const [selectedEdge, setSelectedEdge] = useState(null);

  // Active state modifications
  const [disabledTransitions, setDisabledTransitions] = useState([]);
  const [addedTransitions, setAddedTransitions] = useState([]);
  const [currentGoal, setCurrentGoal] = useState(null);
  const [currentBadStates, setCurrentBadStates] = useState(null);
  const [hasRunInitialPlan, setHasRunInitialPlan] = useState(false);

  // Handle test case selection
  const handleSelectTestCase = useCallback((tc) => {
    setSelectedTestCase(tc);
    setSelectedNode(null);
    setSelectedEdge(null);
    setDisabledTransitions([]);
    setAddedTransitions([]);
    setCurrentGoal(tc.problem.goalState);
    setCurrentBadStates(tc.problem.badStates);
    setHasRunInitialPlan(false);
  }, []);

  // Run initial plan
  const handleRunPlanner = useCallback(() => {
    if (!selectedTestCase) return;
    plan(selectedTestCase.problem, `Initial plan: ${selectedTestCase.name}`);
    setHasRunInitialPlan(true);
    setDisabledTransitions([]);
    setAddedTransitions([]);
    setCurrentGoal(selectedTestCase.problem.goalState);
    setCurrentBadStates(selectedTestCase.problem.badStates);
  }, [selectedTestCase, plan]);

  // Auto-run initial plan when WASM module becomes ready or test case changes
  useEffect(() => {
    if (wasmStatus === 'ready' && selectedTestCase && !hasRunInitialPlan) {
      handleRunPlanner();
    }
  }, [wasmStatus, selectedTestCase, hasRunInitialPlan, handleRunPlanner]);

  // Handle replanning actions
  const handleReplan = useCallback((action, label) => {
    if (action.type === 'disableTransition') {
      setDisabledTransitions(prev => [...prev, action.transitionId]);
    } else if (action.type === 'addTransition') {
      setAddedTransitions(prev => [...prev, action.transition]);
    } else if (action.type === 'changeGoal') {
      setCurrentGoal(action.newGoal);
    } else if (action.type === 'changeBadStates') {
      setCurrentBadStates(action.badStates);
    }

    replan(action, label);
  }, [replan]);

  const activeProblem = selectedTestCase?.problem;

  return (
    <div className="app-layout">
      {/* Top Header */}
      <header className="app-header">
        <div className="app-header__brand">
          <div className="app-header__logo">LPA*</div>
          <div>
            <h1 className="app-header__title">Safe Semantic Planner</h1>
            <div className="app-header__subtitle">Lifelong Planning A* Engine — WASM Executable</div>
          </div>
        </div>
        <div className="app-header__actions">
          {selectedTestCase && (
            <span className="badge badge-accent">
              Test Case #{selectedTestCase.id}: {selectedTestCase.name}
            </span>
          )}
          <ThemeToggle />
        </div>
      </header>

      {/* Sidebar Navigation */}
      <Sidebar
        selectedTestCase={selectedTestCase}
        onSelectTestCase={handleSelectTestCase}
        onRunPlanner={handleRunPlanner}
        isRunning={isRunning}
        wasmStatus={wasmStatus}
        hasResult={result !== null}
      />

      {/* Main Canvas Area */}
      <main className="canvas-area">
        <GraphCanvas
          problem={activeProblem}
          result={result}
          selectedNode={selectedNode}
          selectedEdge={selectedEdge}
          onSelectNode={setSelectedNode}
          onSelectEdge={setSelectedEdge}
          disabledTransitions={disabledTransitions}
          addedTransitions={addedTransitions}
          currentGoal={currentGoal}
          currentBadStates={currentBadStates}
        />
      </main>

      {/* Right Results & Actions Panel */}
      <aside className="results-panel">
        <div className="results-panel__section">
          <ActionPanel
            problem={activeProblem}
            selectedTestCase={selectedTestCase}
            onReplan={handleReplan}
            selectedNode={selectedNode}
            selectedEdge={selectedEdge}
            disabledTransitions={disabledTransitions}
            addedTransitions={addedTransitions}
            currentGoal={currentGoal}
            currentBadStates={currentBadStates}
            hasRunInitialPlan={hasRunInitialPlan}
          />

          <MetricsBar result={result} />
        </div>

        <ResultsPanel
          result={result}
          selectedTestCase={selectedTestCase}
          history={history}
        />

        <div className="results-panel__section">
          <DataTable problem={activeProblem} addedTransitions={addedTransitions} />
        </div>
      </aside>
    </div>
  );
}
