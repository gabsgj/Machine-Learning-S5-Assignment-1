import { useState } from 'react';

/**
 * MetricsBar — optional score calculation panel using Score(P) = αG - βC + γD + δR
 * Allows real-time weight adjustment to see how objective function changes.
 */
export function MetricsBar({ result }) {
  const [alpha, setAlpha] = useState(1.0); // Goal weight
  const [beta, setBeta] = useState(1.0);  // Cost weight
  const [gamma, setGamma] = useState(1.0); // Safety/Distance weight
  const [delta, setDelta] = useState(1.0); // Reliability weight

  if (!result || !result.success) return null;

  const G = 1.0; // Goal reached
  const C = result.totalCost;
  const D = result.safetyScore === Infinity ? 10.0 : result.safetyScore;
  const R = 1.0; // Average reliability

  const score = (alpha * G) - (beta * C) + (gamma * D) + (delta * R);

  return (
    <div className="card" style={{ padding: 12, marginBottom: 12 }}>
      <div className="label">Objective Function Score</div>
      <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 8 }}>
        <span className="mono" style={{ fontSize: 'var(--text-xs)', color: 'var(--color-text-muted)' }}>
          Score = αG − βC + γD + δR
        </span>
        <span className="mono text-accent" style={{ fontSize: 'var(--text-lg)', fontWeight: 700 }}>
          {score.toFixed(2)}
        </span>
      </div>

      <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 8 }}>
        <div>
          <label className="label" style={{ fontSize: '10px' }}>α (Goal): {alpha}</label>
          <input
            type="range"
            min="0"
            max="3"
            step="0.5"
            value={alpha}
            onChange={e => setAlpha(parseFloat(e.target.value))}
            style={{ width: '100%' }}
          />
        </div>
        <div>
          <label className="label" style={{ fontSize: '10px' }}>β (Cost): {beta}</label>
          <input
            type="range"
            min="0"
            max="3"
            step="0.5"
            value={beta}
            onChange={e => setBeta(parseFloat(e.target.value))}
            style={{ width: '100%' }}
          />
        </div>
        <div>
          <label className="label" style={{ fontSize: '10px' }}>γ (Safety): {gamma}</label>
          <input
            type="range"
            min="0"
            max="3"
            step="0.5"
            value={gamma}
            onChange={e => setGamma(parseFloat(e.target.value))}
            style={{ width: '100%' }}
          />
        </div>
        <div>
          <label className="label" style={{ fontSize: '10px' }}>δ (Reliability): {delta}</label>
          <input
            type="range"
            min="0"
            max="3"
            step="0.5"
            value={delta}
            onChange={e => setDelta(parseFloat(e.target.value))}
            style={{ width: '100%' }}
          />
        </div>
      </div>
    </div>
  );
}
