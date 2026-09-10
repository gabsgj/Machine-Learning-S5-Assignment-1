import { useEffect, useRef, useCallback } from 'react';
import cytoscape from 'cytoscape';

/**
 * GraphCanvas — renders the planning graph using Cytoscape.js.
 * Uses ResizeObserver for responsive auto-fit and updates elements in-place
 * to avoid canvas destruction/blanking bugs.
 */
export function GraphCanvas({
  problem,
  result,
  selectedNode,
  selectedEdge,
  onSelectNode,
  onSelectEdge,
  disabledTransitions = [],
  addedTransitions = [],
  currentGoal = null,
  currentBadStates = null,
}) {
  const containerRef = useRef(null);
  const cyRef = useRef(null);

  const goalState = currentGoal ?? problem?.goalState;
  const badStates = currentBadStates ?? problem?.badStates ?? [];

  // Helper to read CSS variables with fallback values
  const getColor = useCallback((varName, fallback) => {
    if (typeof window === 'undefined') return fallback;
    const val = getComputedStyle(document.documentElement).getPropertyValue(varName).trim();
    return val || fallback;
  }, []);

  // Build element objects for Cytoscape
  const buildElements = useCallback(() => {
    if (!problem || !problem.states || problem.states.length === 0) return [];

    const elements = [];

    const xs = problem.states.map(s => s.embedding[0]);
    const ys = problem.states.map(s => s.embedding[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const rangeX = (maxX - minX) || 1;
    const rangeY = (maxY - minY) || 1;
    const padding = 60;

    // Nodes
    for (const s of problem.states) {
      const sId = Number(s.id);
      const initId = Number(problem.initialState);
      const gId = Number(goalState);

      const normX = (s.embedding[0] - minX) / rangeX;
      const normY = rangeY === 0 ? 0.5 : (s.embedding[1] - minY) / rangeY;

      // Vertical offset for 1D graphs (all y=0)
      const yJitter = rangeY === 0 ? (sId % 2 === 0 ? -40 : 40) : 0;

      let nodeType = 'normal';
      if (sId === initId) nodeType = 'start';
      else if (sId === gId) nodeType = 'goal';
      if (badStates.map(Number).includes(sId)) nodeType = 'bad';

      const isOnPath = result?.success && result.statePath.map(Number).includes(sId);

      elements.push({
        group: 'nodes',
        data: {
          id: `n${sId}`,
          stateId: sId,
          label: `S${sId}`,
          nodeType,
          onPath: isOnPath ? 'yes' : 'no',
        },
        position: {
          x: padding + normX * 480 + 40,
          y: padding + normY * 300 + yJitter + 40,
        },
      });
    }

    // Edges
    const allTransitions = [...problem.transitions, ...addedTransitions];
    const disabledSet = disabledTransitions.map(Number);
    const pathTransitions = (result?.transitionPath ?? []).map(Number);

    for (const t of allTransitions) {
      const tId = Number(t.id);
      const isDisabled = disabledSet.includes(tId);
      const isOnPath = result?.success && pathTransitions.includes(tId);
      const isAdded = addedTransitions.some(at => Number(at.id) === tId);

      elements.push({
        group: 'edges',
        data: {
          id: `e${tId}`,
          transitionId: tId,
          source: `n${Number(t.from)}`,
          target: `n${Number(t.to)}`,
          label: `${t.cost.toFixed(1)}`,
          cost: t.cost,
          isDisabled: isDisabled ? 'yes' : 'no',
          onPath: isOnPath ? 'yes' : 'no',
          isAdded: isAdded ? 'yes' : 'no',
          fromState: Number(t.from),
          toState: Number(t.to),
        },
      });
    }

    return elements;
  }, [problem, result, badStates, goalState, disabledTransitions, addedTransitions]);

  // Create Cytoscape instance once when container mounts or problem changes
  useEffect(() => {
    if (!containerRef.current || !problem) return;

    const nodeBg = getColor('--color-node', '#8B949E');
    const startBg = getColor('--color-node-start', '#22D3EE');
    const goalBg = getColor('--color-node-goal', '#34D399');
    const badBg = getColor('--color-node-bad', '#FB923C');
    const pathColor = getColor('--color-accent', '#22D3EE');
    const edgeColor = getColor('--color-edge', '#30363D');
    const disabledColor = getColor('--color-error', '#F87171');
    const bgBase = getColor('--color-bg-base', '#0E1117');
    const textColor = getColor('--color-text', '#E6EDF3');

    const cy = cytoscape({
      container: containerRef.current,
      elements: buildElements(),
      userZoomingEnabled: true,
      userPanningEnabled: true,
      boxSelectionEnabled: false,
      autoungrabify: false,
      style: [
        {
          selector: 'node',
          style: {
            'label': 'data(label)',
            'width': 48,
            'height': 48,
            'font-family': 'sans-serif',
            'font-size': '13px',
            'font-weight': 'bold',
            'text-valign': 'center',
            'text-halign': 'center',
            'color': '#FFFFFF',
            'background-color': nodeBg,
            'border-width': 3,
            'border-color': nodeBg,
            'text-outline-width': 1,
            'text-outline-color': '#000000',
          },
        },
        {
          selector: 'node[nodeType = "start"]',
          style: {
            'background-color': startBg,
            'border-color': startBg,
            'width': 54,
            'height': 54,
          },
        },
        {
          selector: 'node[nodeType = "goal"]',
          style: {
            'background-color': goalBg,
            'border-color': goalBg,
            'width': 54,
            'height': 54,
            'shape': 'diamond',
          },
        },
        {
          selector: 'node[nodeType = "bad"]',
          style: {
            'background-color': badBg,
            'border-color': '#EF4444',
            'border-width': 4,
            'border-style': 'dashed',
          },
        },
        {
          selector: 'node[onPath = "yes"]',
          style: {
            'border-color': pathColor,
            'border-width': 5,
            'overlay-color': pathColor,
            'overlay-padding': 4,
            'overlay-opacity': 0.2,
          },
        },
        {
          selector: 'edge',
          style: {
            'label': 'data(label)',
            'font-family': 'monospace',
            'font-size': '11px',
            'font-weight': 'bold',
            'color': textColor,
            'text-background-color': bgBase,
            'text-background-opacity': 0.9,
            'text-background-padding': '4px',
            'text-background-shape': 'roundrectangle',
            'width': 3,
            'line-color': edgeColor,
            'target-arrow-color': edgeColor,
            'target-arrow-shape': 'triangle',
            'curve-style': 'bezier',
            'arrow-scale': 1.2,
          },
        },
        {
          selector: 'edge[onPath = "yes"]',
          style: {
            'line-color': pathColor,
            'target-arrow-color': pathColor,
            'width': 5,
            'z-index': 10,
          },
        },
        {
          selector: 'edge[isDisabled = "yes"]',
          style: {
            'line-color': disabledColor,
            'target-arrow-color': disabledColor,
            'line-style': 'dashed',
            'opacity': 0.4,
          },
        },
        {
          selector: 'edge[isAdded = "yes"]',
          style: {
            'line-style': 'dashed',
            'line-color': startBg,
            'target-arrow-color': startBg,
          },
        },
      ],
      layout: { name: 'preset' },
    });

    // Tap handlers
    cy.on('tap', 'node', (evt) => {
      const stateId = evt.target.data('stateId');
      onSelectNode?.(stateId);
    });

    cy.on('tap', 'edge', (evt) => {
      const data = evt.target.data();
      onSelectEdge?.({
        transitionId: data.transitionId,
        fromState: data.fromState,
        toState: data.toState,
        cost: data.cost,
      });
    });

    cy.on('tap', (evt) => {
      if (evt.target === cy) {
        onSelectNode?.(null);
        onSelectEdge?.(null);
      }
    });

    cyRef.current = cy;

    // ResizeObserver ensures Cytoscape resizes and fits whenever container dimensions change
    const resizeObserver = new ResizeObserver(() => {
      if (cyRef.current) {
        cyRef.current.resize();
        cyRef.current.fit(undefined, 50);
      }
    });

    resizeObserver.observe(containerRef.current);

    // Initial fit after DOM paint
    setTimeout(() => {
      if (cyRef.current) {
        cyRef.current.resize();
        cyRef.current.fit(undefined, 50);
      }
    }, 50);

    return () => {
      resizeObserver.disconnect();
      if (cyRef.current) {
        cyRef.current.destroy();
        cyRef.current = null;
      }
    };
  }, [problem?.initialState]); // Re-create only if problem changes

  // Update elements in-place when result, badStates, goal, etc. change
  useEffect(() => {
    if (!cyRef.current) return;

    const newElements = buildElements();
    cyRef.current.json({ elements: newElements });
    cyRef.current.fit(undefined, 50);
  }, [buildElements]);

  if (!problem) {
    return (
      <div className="canvas-area">
        <div className="empty-state">
          <div className="empty-state__icon">◇</div>
          <div className="empty-state__text">Select a test case to visualize the graph</div>
        </div>
      </div>
    );
  }

  return (
    <div className="canvas-area" style={{ width: '100%', height: '100%', position: 'relative' }}>
      <div
        ref={containerRef}
        className="canvas-container"
        style={{
          position: 'absolute',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          width: '100%',
          height: '100%',
        }}
      />
      <div className="canvas-legend">
        <div className="canvas-legend__item">
          <div className="canvas-legend__dot" style={{ background: '#22D3EE' }} />
          <span>Start</span>
        </div>
        <div className="canvas-legend__item">
          <div className="canvas-legend__dot" style={{ background: '#34D399' }} />
          <span>Goal</span>
        </div>
        <div className="canvas-legend__item">
          <div className="canvas-legend__dot" style={{ background: '#FB923C' }} />
          <span>Bad State</span>
        </div>
        <div className="canvas-legend__item">
          <div className="canvas-legend__dot" style={{ background: '#22D3EE' }} />
          <span>Active Path</span>
        </div>
      </div>
    </div>
  );
}
