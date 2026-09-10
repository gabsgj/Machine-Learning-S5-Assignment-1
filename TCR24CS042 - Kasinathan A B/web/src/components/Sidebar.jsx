import { testCases } from '../data/test-cases';

/**
 * Sidebar — test case picker, run planner button, and contextual info.
 */
export function Sidebar({
  selectedTestCase,
  onSelectTestCase,
  onRunPlanner,
  isRunning,
  wasmStatus,
  hasResult,
}) {
  return (
    <div className="sidebar">
      {/* WASM Status */}
      <div className="sidebar__section">
        <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 6 }}>
          <span
            className={`status-dot ${
              wasmStatus === 'ready' ? 'status-dot--ready' :
              wasmStatus === 'error' ? 'status-dot--error' :
              'status-dot--loading'
            }`}
          />
          <span className="text-muted" style={{ fontSize: 'var(--text-xs)' }}>
            {wasmStatus === 'ready' ? 'WASM Engine Ready' :
             wasmStatus === 'error' ? 'WASM Load Failed' :
             'Loading WASM...'}
          </span>
        </div>
      </div>

      {/* Test Cases */}
      <div className="sidebar__section" style={{ flex: 1 }}>
        <div className="sidebar__section-title">Test Cases</div>
        {testCases.map(tc => (
          <div
            key={tc.id}
            className={`test-case-card ${selectedTestCase?.id === tc.id ? 'active' : ''}`}
            onClick={() => onSelectTestCase(tc)}
          >
            <div className="test-case-card__name">
              <span className="mono" style={{ color: 'var(--color-accent)', marginRight: 6, fontSize: 'var(--text-xs)' }}>
                #{tc.id}
              </span>
              {tc.name}
            </div>
            <div className="test-case-card__desc">{tc.description}</div>
          </div>
        ))}
      </div>

      {/* Run Button */}
      <div className="sidebar__section">
        <button
          className="btn btn-primary"
          style={{ width: '100%' }}
          onClick={onRunPlanner}
          disabled={!selectedTestCase || wasmStatus !== 'ready' || isRunning}
        >
          {isRunning ? (
            <>
              <span className="animate-pulse">●</span>
              Planning...
            </>
          ) : (
            <>
              <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor"><polygon points="5,3 19,12 5,21"/></svg>
              Run LPA* Planner
            </>
          )}
        </button>
        {selectedTestCase && (
          <div style={{ marginTop: 8, fontSize: 'var(--text-xs)', color: 'var(--color-text-muted)' }}>
            Graph: {selectedTestCase.problem.states.length} states, {selectedTestCase.problem.transitions.length} transitions
            {selectedTestCase.problem.badStates.length > 0 && (
              <>, {selectedTestCase.problem.badStates.length} bad state{selectedTestCase.problem.badStates.length > 1 ? 's' : ''}</>
            )}
          </div>
        )}
      </div>
    </div>
  );
}
