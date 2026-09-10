import { useState } from 'react';

/**
 * ActionPanel — provides controls for the four replanning scenarios:
 * 1. Disable transition (onTransitionUnavailable)
 * 2. Add transition (onTransitionAdded)
 * 3. Change bad states (onBadStatesChanged)
 * 4. Change goal state (onGoalChanged)
 */
export function ActionPanel({
  problem,
  selectedTestCase,
  onReplan,
  selectedNode,
  selectedEdge,
  disabledTransitions = [],
  addedTransitions = [],
  currentGoal,
  currentBadStates,
  hasRunInitialPlan,
}) {
  const [activeTab, setActiveTab] = useState('preset'); // 'preset' | 'disable' | 'add' | 'bad' | 'goal'

  // Form states for adding transition
  const [addFrom, setAddFrom] = useState('1');
  const [addTo, setAddTo] = useState('4');
  const [addCost, setAddCost] = useState('1.5');
  const [addId, setAddId] = useState('99');

  if (!hasRunInitialPlan) {
    return (
      <div className="card" style={{ padding: 12, marginBottom: 12 }}>
        <div style={{ fontSize: 'var(--text-xs)', color: 'var(--color-text-muted)', textAlign: 'center' }}>
          Run initial plan first to enable incremental replanning controls.
        </div>
      </div>
    );
  }

  const replanPreset = selectedTestCase?.replanAction;

  const handleDisableEdge = (edgeToDisable) => {
    if (!edgeToDisable) return;
    onReplan(
      {
        type: 'disableTransition',
        transitionId: edgeToDisable.transitionId,
        fromState: edgeToDisable.fromState,
        toState: edgeToDisable.toState,
      },
      `Disable edge #${edgeToDisable.transitionId} (${edgeToDisable.fromState}→${edgeToDisable.toState})`
    );
  };

  const handleAddTransitionSubmit = (e) => {
    e.preventDefault();
    const from = parseInt(addFrom, 10);
    const to = parseInt(addTo, 10);
    const cost = parseFloat(addCost);
    const id = parseInt(addId, 10);

    if (isNaN(from) || isNaN(to) || isNaN(cost) || isNaN(id)) return;

    onReplan(
      {
        type: 'addTransition',
        transition: {
          id,
          from,
          to,
          cost,
          safety: 1.0,
          reliability: 1.0,
          available: true,
        },
      },
      `Add shortcut S${from}→S${to} (cost ${cost})`
    );
  };

  const handleGoalChangeSubmit = (newGoalId) => {
    onReplan(
      {
        type: 'changeGoal',
        newGoal: newGoalId,
      },
      `Goal changed to state S${newGoalId}`
    );
  };

  const handleBadStateToggle = (stateId) => {
    const updated = currentBadStates.includes(stateId)
      ? currentBadStates.filter(id => id !== stateId)
      : [...currentBadStates, stateId];

    onReplan(
      {
        type: 'changeBadStates',
        badStates: updated,
      },
      `Bad states updated: [${updated.join(', ')}]`
    );
  };

  return (
    <div className="card" style={{ padding: 12, marginBottom: 12 }}>
      <div className="label" style={{ marginBottom: 8 }}>Interactive Replanning</div>

      {/* Preset Action Button if Test Case has one */}
      {replanPreset && (
        <div style={{ marginBottom: 10 }}>
          <button
            className="btn btn-secondary btn-sm"
            style={{ width: '100%', borderColor: 'var(--color-accent)' }}
            onClick={() => {
              if (replanPreset.type === 'disableTransition') {
                onReplan(
                  {
                    type: 'disableTransition',
                    transitionId: replanPreset.transitionId,
                    fromState: replanPreset.fromState,
                    toState: replanPreset.toState,
                  },
                  replanPreset.label
                );
              } else if (replanPreset.type === 'changeGoal') {
                onReplan(
                  {
                    type: 'changeGoal',
                    newGoal: replanPreset.newGoal,
                  },
                  replanPreset.label
                );
              } else if (replanPreset.type === 'addTransition') {
                onReplan(
                  {
                    type: 'addTransition',
                    transition: replanPreset.transition,
                  },
                  replanPreset.label
                );
              }
            }}
          >
            ⚡ Preset Replan: {replanPreset.label}
          </button>
        </div>
      )}

      {/* Tab Selectors */}
      <div style={{ display: 'flex', gap: 4, marginBottom: 10, flexWrap: 'wrap' }}>
        <button
          className={`btn btn-sm ${activeTab === 'disable' ? 'btn-primary' : 'btn-ghost'}`}
          onClick={() => setActiveTab('disable')}
        >
          Disable Edge
        </button>
        <button
          className={`btn btn-sm ${activeTab === 'add' ? 'btn-primary' : 'btn-ghost'}`}
          onClick={() => setActiveTab('add')}
        >
          Add Edge
        </button>
        <button
          className={`btn btn-sm ${activeTab === 'bad' ? 'btn-primary' : 'btn-ghost'}`}
          onClick={() => setActiveTab('bad')}
        >
          Bad States
        </button>
        <button
          className={`btn btn-sm ${activeTab === 'goal' ? 'btn-primary' : 'btn-ghost'}`}
          onClick={() => setActiveTab('goal')}
        >
          Set Goal
        </button>
      </div>

      {/* Tab Content: Disable Edge */}
      {activeTab === 'disable' && (
        <div className="animate-fadeIn">
          {selectedEdge ? (
            <div>
              <div style={{ fontSize: 'var(--text-xs)', marginBottom: 6 }}>
                Selected Edge: <strong className="mono">#{selectedEdge.transitionId} (S{selectedEdge.fromState} → S{selectedEdge.toState})</strong>
              </div>
              <button
                className="btn btn-danger btn-sm"
                style={{ width: '100%' }}
                onClick={() => handleDisableEdge(selectedEdge)}
              >
                Disable Edge #{selectedEdge.transitionId}
              </button>
            </div>
          ) : (
            <div>
              <div style={{ fontSize: 'var(--text-xs)', color: 'var(--color-text-muted)', marginBottom: 6 }}>
                Click an edge on the graph canvas or select below:
              </div>
              <div style={{ display: 'flex', flexDirection: 'column', gap: 4 }}>
                {problem?.transitions.map(t => (
                  <button
                    key={t.id}
                    className="btn btn-ghost btn-sm"
                    style={{ justifyContent: 'space-between' }}
                    onClick={() => handleDisableEdge({ transitionId: t.id, fromState: t.from, toState: t.to })}
                    disabled={disabledTransitions.includes(t.id)}
                  >
                    <span>Edge #{t.id} (S{t.from} → S{t.to})</span>
                    <span className="mono">cost {t.cost}</span>
                  </button>
                ))}
              </div>
            </div>
          )}
        </div>
      )}

      {/* Tab Content: Add Edge */}
      {activeTab === 'add' && (
        <form onSubmit={handleAddTransitionSubmit} className="inline-form animate-fadeIn">
          <div className="inline-form__row">
            <div className="inline-form__field">
              <label className="label">From State</label>
              <select className="select" value={addFrom} onChange={e => setAddFrom(e.target.value)}>
                {problem?.states.map(s => (
                  <option key={s.id} value={s.id}>S{s.id}</option>
                ))}
              </select>
            </div>
            <div className="inline-form__field">
              <label className="label">To State</label>
              <select className="select" value={addTo} onChange={e => setAddTo(e.target.value)}>
                {problem?.states.map(s => (
                  <option key={s.id} value={s.id}>S{s.id}</option>
                ))}
              </select>
            </div>
          </div>
          <div className="inline-form__row">
            <div className="inline-form__field">
              <label className="label">Cost</label>
              <input
                type="number"
                step="0.1"
                className="input input-mono"
                value={addCost}
                onChange={e => setAddCost(e.target.value)}
              />
            </div>
            <div className="inline-form__field">
              <label className="label">Edge ID</label>
              <input
                type="number"
                className="input input-mono"
                value={addId}
                onChange={e => setAddId(e.target.value)}
              />
            </div>
          </div>
          <button type="submit" className="btn btn-primary btn-sm" style={{ marginTop: 4 }}>
            Add & Replan
          </button>
        </form>
      )}

      {/* Tab Content: Bad States */}
      {activeTab === 'bad' && (
        <div className="animate-fadeIn">
          <div style={{ fontSize: 'var(--text-xs)', color: 'var(--color-text-muted)', marginBottom: 8 }}>
            Toggle state safety status:
          </div>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6 }}>
            {problem?.states.map(s => {
              const sId = Number(s.id);
              const isBad = currentBadStates.map(Number).includes(sId);
              return (
                <button
                  key={sId}
                  className={`btn btn-sm ${isBad ? 'btn-danger' : 'btn-secondary'}`}
                  onClick={() => handleBadStateToggle(sId)}
                >
                  S{sId} {isBad ? '(BAD)' : '(Safe)'}
                </button>
              );
            })}
          </div>
        </div>
      )}

      {/* Tab Content: Set Goal */}
      {activeTab === 'goal' && (
        <div className="animate-fadeIn">
          <div style={{ fontSize: 'var(--text-xs)', color: 'var(--color-text-muted)', marginBottom: 8 }}>
            Select a new target goal state:
          </div>
          <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: 6 }}>
            {problem?.states.map(s => {
              const isCurrentGoal = currentGoal === s.id;
              return (
                <button
                  key={s.id}
                  className={`btn btn-sm ${isCurrentGoal ? 'btn-primary' : 'btn-secondary'}`}
                  disabled={isCurrentGoal}
                  onClick={() => handleGoalChangeSubmit(s.id)}
                >
                  S{s.id} {isCurrentGoal ? '(Current Goal)' : ''}
                </button>
              );
            })}
          </div>
        </div>
      )}
    </div>
  );
}
