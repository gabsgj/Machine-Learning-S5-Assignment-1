/**
 * ResultsPanel — displays PlanningResult details:
 * - Success/failure status badge
 * - State path breadcrumbs
 * - Transition path IDs
 * - Total cost & safety score
 * - States expanded count
 * - Measured planning/replanning time
 */
export function ResultsPanel({ result, selectedTestCase, history = [] }) {
  if (!result) {
    return (
      <div className="results-panel">
        <div className="results-panel__section">
          <div className="results-panel__section-title">Planning Results</div>
          <div className="empty-state" style={{ padding: '20px 0' }}>
            <div className="empty-state__icon">📊</div>
            <div className="empty-state__text">Run the planner to view execution metrics</div>
          </div>
        </div>
      </div>
    );
  }

  const { success, statePath, transitionPath, totalCost, safetyScore, statesExpanded, timeMs } = result;

  return (
    <div className="results-panel animate-fadeIn">
      {/* Status Header */}
      <div className="results-panel__section">
        <div className="results-panel__section-title">Execution Result</div>
        <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 12 }}>
          <span className={`badge ${success ? 'badge-success' : 'badge-error'}`}>
            {success ? '✓ Path Found' : '✕ No Path Found'}
          </span>
          <span className="mono text-muted" style={{ fontSize: 'var(--text-xs)' }}>
            {timeMs !== undefined ? `${timeMs.toFixed(2)} ms` : 'N/A'}
          </span>
        </div>

        {/* Path Breadcrumbs */}
        {success && statePath.length > 0 && (
          <div>
            <div className="label">State Path ({statePath.length} states)</div>
            <div className="path-breadcrumb" style={{ marginTop: 6, marginBottom: 12 }}>
              {statePath.map((s, idx) => (
                <span key={idx} style={{ display: 'inline-flex', alignItems: 'center' }}>
                  <span className="node">S{s}</span>
                  {idx < statePath.length - 1 && <span className="arrow">→</span>}
                </span>
              ))}
            </div>
          </div>
        )}
      </div>

      {/* Primary Metrics */}
      <div className="results-panel__section">
        <div className="results-panel__section-title">Key Performance Indicators</div>

        <div className="metric-row">
          <span className="metric-row__label">Total Path Cost</span>
          <span className="metric-row__value text-accent">
            {success ? totalCost.toFixed(2) : '∞'}
          </span>
        </div>

        <div className="metric-row">
          <span className="metric-row__label">Safety Score (Min Dist to Danger)</span>
          <span className="metric-row__value text-success">
            {success ? (safetyScore === Infinity ? 'Safe (∞)' : safetyScore.toFixed(2)) : 'N/A'}
          </span>
        </div>

        <div className="metric-row">
          <span className="metric-row__label">States Expanded</span>
          <span className="metric-row__value text-secondary">
            {statesExpanded ?? 0}
          </span>
        </div>

        <div className="metric-row">
          <span className="metric-row__label">Execution Time</span>
          <span className="metric-row__value mono">
            {timeMs !== undefined ? `${timeMs.toFixed(3)} ms` : 'N/A'}
          </span>
        </div>
      </div>

      {/* Transition Path Details */}
      {success && transitionPath.length > 0 && (
        <div className="results-panel__section">
          <div className="results-panel__section-title">Transition Sequence</div>
          <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap' }}>
            {transitionPath.map((tId, idx) => (
              <span key={idx} className="badge badge-accent">
                Edge #{tId}
              </span>
            ))}
          </div>
        </div>
      )}

      {/* History Log */}
      {history.length > 0 && (
        <div className="results-panel__section" style={{ flex: 1 }}>
          <div className="results-panel__section-title">Replanning Log</div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: 8 }}>
            {history.map((item, idx) => (
              <div key={idx} className="card card-raised" style={{ padding: '8px 10px' }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 'var(--text-xs)' }}>
                  <span style={{ fontWeight: 600, color: 'var(--color-text)' }}>{item.label}</span>
                  <span className="mono text-muted">{item.result.timeMs?.toFixed(2)} ms</span>
                </div>
                <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: 'var(--text-xs)', marginTop: 4 }}>
                  <span className="text-muted">Expanded: {item.result.statesExpanded}</span>
                  <span className={item.result.success ? 'text-success' : 'text-error'}>
                    Cost: {item.result.success ? item.result.totalCost.toFixed(1) : 'Failed'}
                  </span>
                </div>
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
}
