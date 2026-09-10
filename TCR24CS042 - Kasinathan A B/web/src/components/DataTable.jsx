import { useState } from 'react';

/**
 * DataTable — displays state embeddings & transition lists in tabular format.
 */
export function DataTable({ problem, addedTransitions = [] }) {
  const [tab, setTab] = useState('states');

  if (!problem) return null;

  const allTransitions = [...problem.transitions, ...addedTransitions];

  return (
    <div className="card" style={{ padding: 12, marginTop: 12 }}>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 8 }}>
        <div className="label">Graph Data Table</div>
        <div style={{ display: 'flex', gap: 4 }}>
          <button
            className={`btn btn-xs ${tab === 'states' ? 'btn-primary' : 'btn-ghost'}`}
            onClick={() => setTab('states')}
          >
            States ({problem.states.length})
          </button>
          <button
            className={`btn btn-xs ${tab === 'transitions' ? 'btn-primary' : 'btn-ghost'}`}
            onClick={() => setTab('transitions')}
          >
            Transitions ({allTransitions.length})
          </button>
        </div>
      </div>

      {tab === 'states' && (
        <div style={{ maxHeight: 150, overflowY: 'auto' }}>
          <table style={{ width: '100%', fontSize: 'var(--text-xs)', borderCollapse: 'collapse' }}>
            <thead>
              <tr style={{ borderBottom: '1px solid var(--color-border)', textAlign: 'left' }}>
                <th style={{ padding: 4 }}>ID</th>
                <th style={{ padding: 4 }}>Embedding (Coordinates)</th>
              </tr>
            </thead>
            <tbody>
              {problem.states.map(s => (
                <tr key={s.id} style={{ borderBottom: '1px solid var(--color-border)' }}>
                  <td className="mono" style={{ padding: 4 }}>S{s.id}</td>
                  <td className="mono text-muted" style={{ padding: 4 }}>[{s.embedding.join(', ')}]</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}

      {tab === 'transitions' && (
        <div style={{ maxHeight: 150, overflowY: 'auto' }}>
          <table style={{ width: '100%', fontSize: 'var(--text-xs)', borderCollapse: 'collapse' }}>
            <thead>
              <tr style={{ borderBottom: '1px solid var(--color-border)', textAlign: 'left' }}>
                <th style={{ padding: 4 }}>ID</th>
                <th style={{ padding: 4 }}>From → To</th>
                <th style={{ padding: 4 }}>Cost</th>
                <th style={{ padding: 4 }}>Available</th>
              </tr>
            </thead>
            <tbody>
              {allTransitions.map(t => (
                <tr key={t.id} style={{ borderBottom: '1px solid var(--color-border)' }}>
                  <td className="mono" style={{ padding: 4 }}>#{t.id}</td>
                  <td className="mono" style={{ padding: 4 }}>S{t.from} → S{t.to}</td>
                  <td className="mono text-accent" style={{ padding: 4 }}>{t.cost}</td>
                  <td style={{ padding: 4 }}>
                    <span className={`badge ${t.available ? 'badge-success' : 'badge-error'}`}>
                      {t.available ? 'Yes' : 'No'}
                    </span>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
}
