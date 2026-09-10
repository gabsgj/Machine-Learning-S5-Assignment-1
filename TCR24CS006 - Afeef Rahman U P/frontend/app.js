/**
 * Safe Semantic Planner - Interactive Web Visualizer Application
 * Canvas 2D/3D state space renderer with particle flows, live LPA* replanning & step animation
 */

document.addEventListener("DOMContentLoaded", () => {
    // --- Canvas & Context ---
    const canvas = document.getElementById("graphCanvas");
    const ctx = canvas.getContext("2d");

    // --- Core State & Engines ---
    const planner = new LPAStarPlannerJS();
    let currentProblem = null;
    let planningResult = null;
    let activeTool = "select"; // 'select', 'addState', 'addTransition', 'toggleBad', 'toggleEdge', 'setStart', 'setGoal'
    let selectedStateId = null;
    let hoveredStateId = null;
    let hoveredEdgeId = null;
    let transitionFromId = null;
    let isDragging = false;
    let dragTarget = null;
    let animationStepInterval = null;
    let particleOffset = 0;

    // Viewport Transform (Pan & Zoom)
    let zoom = 1.0;
    let panX = 0;
    let panY = 0;
    let isPanning = false;
    let panStartX = 0;
    let panStartY = 0;

    // --- DOM Elements ---
    const scenarioSelect = document.getElementById("scenarioSelect");
    const scenarioDescription = document.getElementById("scenarioDescription");
    const btnPlanCold = document.getElementById("btnPlanCold");
    const btnReplanInc = document.getElementById("btnReplanInc");
    const btnStep = document.getElementById("btnStep");
    const btnPlay = document.getElementById("btnPlay");
    const btnReset = document.getElementById("btnReset");
    const btnClear = document.getElementById("btnClear");
    const btnExportJson = document.getElementById("btnExportJson");
    const fileImportInput = document.getElementById("fileImportInput");

    // Sliders
    const sliderAlpha = document.getElementById("sliderAlpha");
    const sliderBeta = document.getElementById("sliderBeta");
    const sliderGamma = document.getElementById("sliderGamma");
    const sliderDelta = document.getElementById("sliderDelta");
    const sliderSafetyMargin = document.getElementById("sliderSafetyMargin");

    const valAlpha = document.getElementById("valAlpha");
    const valBeta = document.getElementById("valBeta");
    const valGamma = document.getElementById("valGamma");
    const valDelta = document.getElementById("valDelta");
    const valSafetyMargin = document.getElementById("valSafetyMargin");

    // Stats Displays
    const statStatus = document.getElementById("statStatus");
    const statCost = document.getElementById("statCost");
    const statSafety = document.getElementById("statSafety");
    const statReliability = document.getElementById("statReliability");
    const statScore = document.getElementById("statScore");
    const statExplored = document.getElementById("statExplored");
    const statTime = document.getElementById("statTime");
    const pathText = document.getElementById("pathText");
    const queueTableBody = document.getElementById("queueTableBody");

    // Resize Canvas to device pixel ratio
    function resizeCanvas() {
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width * window.devicePixelRatio;
        canvas.height = rect.height * window.devicePixelRatio;
        ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
        draw();
    }
    window.addEventListener("resize", resizeCanvas);

    // --- Scenario Loader ---
    function loadScenario(scenarioKey) {
        if (!SCENARIOS[scenarioKey]) return;
        const sc = SCENARIOS[scenarioKey];
        currentProblem = JSON.parse(JSON.stringify(sc));
        
        scenarioDescription.textContent = sc.description;
        document.getElementById("currentScenarioBadge").textContent = sc.name;

        // Apply sliders to problem
        syncWeightsToProblem();
        
        // Load into LPA* engine and run cold plan
        planner.loadProblem(currentProblem);
        planningResult = planner.plan(currentProblem);

        // Center view on graph
        centerGraphInView();
        updateTelemetry();
        draw();
    }

    function syncWeightsToProblem() {
        if (!currentProblem) return;
        currentProblem.alpha = parseFloat(sliderAlpha.value);
        currentProblem.beta = parseFloat(sliderBeta.value);
        currentProblem.gamma = parseFloat(sliderGamma.value);
        currentProblem.delta = parseFloat(sliderDelta.value);
        
        const safetyMarginVal = parseFloat(sliderSafetyMargin.value);
        planner.safetyMargin = safetyMarginVal;
        planner.safetyWeight = currentProblem.gamma > 0 ? currentProblem.gamma * 2.0 : 0.0;
        planner.reliabilityWeight = currentProblem.delta > 0 ? currentProblem.delta : 0.0;
    }

    function centerGraphInView() {
        if (!currentProblem || currentProblem.states.length === 0) return;
        const rect = canvas.getBoundingClientRect();
        
        let minX = INF, maxX = -INF, minY = INF, maxY = -INF;
        currentProblem.states.forEach(s => {
            minX = Math.min(minX, s.embedding[0]);
            maxX = Math.max(maxX, s.embedding[0]);
            minY = Math.min(minY, s.embedding[1]);
            maxY = Math.max(maxY, s.embedding[1]);
        });

        const graphW = Math.max(maxX - minX, 100);
        const graphH = Math.max(maxY - minY, 100);
        const scaleX = (rect.width - 160) / graphW;
        const scaleY = (rect.height - 160) / graphH;
        zoom = Math.min(Math.max(Math.min(scaleX, scaleY), 0.5), 2.0);

        const centerX = (minX + maxX) / 2;
        const centerY = (minY + maxY) / 2;
        panX = (rect.width / 2) - (centerX * zoom);
        panY = (rect.height / 2) - (centerY * zoom);
    }

    // --- Coordinate Transforms ---
    function worldToScreen(wx, wy) {
        return {
            x: wx * zoom + panX,
            y: wy * zoom + panY
        };
    }

    function screenToWorld(sx, sy) {
        return {
            x: (sx - panX) / zoom,
            y: (sy - panY) / zoom
        };
    }

    // --- State & Edge Hit Testing ---
    function findStateAt(wx, wy, radius = 24) {
        if (!currentProblem) return null;
        for (const s of currentProblem.states) {
            const dist = Math.hypot(s.embedding[0] - wx, s.embedding[1] - wy);
            if (dist <= radius / zoom) return s;
        }
        return null;
    }

    function findEdgeAt(wx, wy, threshold = 8) {
        if (!currentProblem) return null;
        for (const t of currentProblem.transitions) {
            const sFrom = currentProblem.states.find(s => s.id === t.from);
            const sTo = currentProblem.states.find(s => s.id === t.to);
            if (sFrom && sTo) {
                const d = distToSegment(wx, wy, sFrom.embedding[0], sFrom.embedding[1], sTo.embedding[0], sTo.embedding[1]);
                if (d <= threshold / zoom) return t;
            }
        }
        return null;
    }

    function distToSegment(px, py, x1, y1, x2, y2) {
        const l2 = (x2 - x1) ** 2 + (y2 - y1) ** 2;
        if (l2 === 0) return Math.hypot(px - x1, py - y1);
        let t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / l2;
        t = Math.max(0, Math.min(1, t));
        return Math.hypot(px - (x1 + t * (x2 - x1)), py - (y1 + t * (y2 - y1)));
    }

    // --- Drawing Engine ---
    function draw() {
        const rect = canvas.getBoundingClientRect();
        ctx.clearRect(0, 0, rect.width, rect.height);

        // Draw Grid
        drawGrid(rect.width, rect.height);

        if (!currentProblem) return;

        const pathSet = new Set();
        if (planningResult && planningResult.success) {
            for (let i = 0; i + 1 < planningResult.statePath.length; i++) {
                pathSet.add(`${planningResult.statePath[i]}->${planningResult.statePath[i+1]}`);
            }
        }

        // 1. Draw Safety Hazard Fields around Bad States
        currentProblem.badStates.forEach(badId => {
            const badState = currentProblem.states.find(s => s.id === badId);
            if (badState) {
                const sp = worldToScreen(badState.embedding[0], badState.embedding[1]);
                const radius = 80 * zoom;

                // Glowing radial danger gradient
                const grad = ctx.createRadialGradient(sp.x, sp.y, 10 * zoom, sp.x, sp.y, radius);
                grad.addColorStop(0, "rgba(244, 63, 94, 0.45)");
                grad.addColorStop(0.5, "rgba(244, 63, 94, 0.15)");
                grad.addColorStop(1, "rgba(244, 63, 94, 0.0)");
                ctx.fillStyle = grad;
                ctx.beginPath();
                ctx.arc(sp.x, sp.y, radius, 0, Math.PI * 2);
                ctx.fill();

                // Dotted safety buffer boundary ring
                ctx.save();
                ctx.strokeStyle = "rgba(244, 63, 94, 0.5)";
                ctx.lineWidth = 1.5;
                ctx.setLineDash([4, 4]);
                ctx.beginPath();
                ctx.arc(sp.x, sp.y, radius * 0.75, 0, Math.PI * 2);
                ctx.stroke();
                ctx.restore();
            }
        });

        // 2. Draw Directed Transitions
        currentProblem.transitions.forEach(t => {
            const fromState = currentProblem.states.find(s => s.id === t.from);
            const toState = currentProblem.states.find(s => s.id === t.to);
            if (!fromState || !toState) return;

            const p1 = worldToScreen(fromState.embedding[0], fromState.embedding[1]);
            const p2 = worldToScreen(toState.embedding[0], toState.embedding[1]);
            const isPath = pathSet.has(`${t.from}->${t.to}`);
            const isHovered = hoveredEdgeId === t.id;

            drawEdge(p1, p2, t, isPath, isHovered);
        });

        // 3. Draw Nodes (States)
        currentProblem.states.forEach(s => {
            const sp = worldToScreen(s.embedding[0], s.embedding[1]);
            const isStart = (s.id === currentProblem.initialState);
            const isGoal = (s.id === currentProblem.goalState);
            const isBad = currentProblem.badStates.includes(s.id);
            const isInPath = planningResult && planningResult.success && planningResult.statePath.includes(s.id);
            const isSelected = (selectedStateId === s.id);
            const isHovered = (hoveredStateId === s.id);

            drawNode(sp, s, isStart, isGoal, isBad, isInPath, isSelected, isHovered);
        });

        // 4. Draw Path Animation Pulse Particles
        if (planningResult && planningResult.success && planningResult.statePath.length > 1) {
            drawPathParticles();
        }
    }

    function drawGrid(w, h) {
        ctx.save();
        ctx.strokeStyle = "rgba(255, 255, 255, 0.03)";
        ctx.lineWidth = 1;
        const gridSize = 40 * zoom;
        const offsetX = panX % gridSize;
        const offsetY = panY % gridSize;

        ctx.beginPath();
        for (let x = offsetX; x < w; x += gridSize) {
            ctx.moveTo(x, 0);
            ctx.lineTo(x, h);
        }
        for (let y = offsetY; y < h; y += gridSize) {
            ctx.moveTo(0, y);
            ctx.lineTo(w, y);
        }
        ctx.stroke();
        ctx.restore();
    }

    function drawEdge(p1, p2, t, isPath, isHovered) {
        ctx.save();
        const angle = Math.atan2(p2.y - p1.y, p2.x - p1.x);
        const nodeRadius = 22 * zoom;

        // Offset from center to node perimeter
        const startX = p1.x + Math.cos(angle) * nodeRadius;
        const startY = p1.y + Math.sin(angle) * nodeRadius;
        const endX = p2.x - Math.cos(angle) * (nodeRadius + 4);
        const endY = p2.y - Math.sin(angle) * (nodeRadius + 4);

        if (!t.available) {
            // Disabled / blocked edge
            ctx.strokeStyle = "rgba(100, 116, 139, 0.35)";
            ctx.lineWidth = 2;
            ctx.setLineDash([4, 4]);
        } else if (isPath) {
            // Active optimal path ribbon
            ctx.strokeStyle = "#38bdf8";
            ctx.lineWidth = isHovered ? 5 : 4;
            ctx.shadowColor = "rgba(56, 189, 248, 0.8)";
            ctx.shadowBlur = 12;
        } else if (isHovered) {
            ctx.strokeStyle = "#a855f7";
            ctx.lineWidth = 3;
            ctx.shadowColor = "rgba(168, 85, 247, 0.6)";
            ctx.shadowBlur = 8;
        } else {
            // Normal transition edge
            ctx.strokeStyle = "rgba(255, 255, 255, 0.18)";
            ctx.lineWidth = 2;
        }

        ctx.beginPath();
        ctx.moveTo(startX, startY);
        ctx.lineTo(endX, endY);
        ctx.stroke();

        // Arrow head
        const headLength = (isPath ? 12 : 9) * zoom;
        ctx.beginPath();
        ctx.moveTo(endX, endY);
        ctx.lineTo(endX - headLength * Math.cos(angle - Math.PI / 6), endY - headLength * Math.sin(angle - Math.PI / 6));
        ctx.lineTo(endX - headLength * Math.cos(angle + Math.PI / 6), endY - headLength * Math.sin(angle + Math.PI / 6));
        ctx.closePath();
        ctx.fillStyle = isPath ? "#38bdf8" : (t.available ? "rgba(255, 255, 255, 0.4)" : "rgba(100, 116, 139, 0.4)");
        ctx.fill();

        // Cost & Reliability Label
        const midX = (startX + endX) / 2 + Math.cos(angle + Math.PI/2) * (12 * zoom);
        const midY = (startY + endY) / 2 + Math.sin(angle + Math.PI/2) * (12 * zoom);
        
        ctx.font = `500 ${Math.max(10 * zoom, 9)}px 'JetBrains Mono', monospace`;
        ctx.fillStyle = isPath ? "#38bdf8" : (t.available ? "#94a3b8" : "#64748b");
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(`c:${t.cost.toFixed(1)}`, midX, midY);

        ctx.restore();
    }

    function drawNode(sp, s, isStart, isGoal, isBad, isInPath, isSelected, isHovered) {
        ctx.save();
        const baseRadius = 20 * zoom;

        // Base Circle
        ctx.beginPath();
        ctx.arc(sp.x, sp.y, baseRadius, 0, Math.PI * 2);

        if (isBad) {
            ctx.fillStyle = "rgba(244, 63, 94, 0.85)";
            ctx.strokeStyle = "#f43f5e";
            ctx.lineWidth = 3;
            ctx.shadowColor = "rgba(244, 63, 94, 0.8)";
            ctx.shadowBlur = 14;
        } else if (isStart) {
            ctx.fillStyle = "rgba(16, 185, 129, 0.85)";
            ctx.strokeStyle = "#10b981";
            ctx.lineWidth = 3;
            ctx.shadowColor = "rgba(16, 185, 129, 0.9)";
            ctx.shadowBlur = 16;
        } else if (isGoal) {
            ctx.fillStyle = "rgba(245, 158, 11, 0.85)";
            ctx.strokeStyle = "#f59e0b";
            ctx.lineWidth = 3;
            ctx.shadowColor = "rgba(245, 158, 11, 0.9)";
            ctx.shadowBlur = 16;
        } else if (isInPath) {
            ctx.fillStyle = "rgba(15, 23, 42, 0.9)";
            ctx.strokeStyle = "#38bdf8";
            ctx.lineWidth = 2.5;
            ctx.shadowColor = "rgba(56, 189, 248, 0.6)";
            ctx.shadowBlur = 10;
        } else {
            ctx.fillStyle = "rgba(15, 23, 42, 0.85)";
            ctx.strokeStyle = "rgba(255, 255, 255, 0.2)";
            ctx.lineWidth = 1.5;
        }

        ctx.fill();
        ctx.stroke();

        if (isSelected || isHovered) {
            ctx.strokeStyle = "#ffffff";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(sp.x, sp.y, baseRadius + 4, 0, Math.PI * 2);
            ctx.stroke();
        }

        // Inner ID
        ctx.fillStyle = "#ffffff";
        ctx.font = `700 ${Math.max(12 * zoom, 10)}px 'Outfit', sans-serif`;
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";
        ctx.fillText(s.id.toString(), sp.x, sp.y);

        // Label above node
        ctx.font = `600 ${Math.max(11 * zoom, 9)}px 'Outfit', sans-serif`;
        ctx.fillStyle = isBad ? "#f43f5e" : (isStart ? "#10b981" : (isGoal ? "#f59e0b" : "#e2e8f0"));
        ctx.fillText(s.label || `S${s.id}`, sp.x, sp.y - baseRadius - (8 * zoom));

        // g and rhs values badge below node
        const gVal = planner.getG(s.id);
        const rhsVal = planner.getRhs(s.id);
        const gStr = gVal >= INF ? "∞" : gVal.toFixed(1);
        const rhsStr = rhsVal >= INF ? "∞" : rhsVal.toFixed(1);
        
        ctx.font = `500 ${Math.max(9 * zoom, 8)}px 'JetBrains Mono', monospace`;
        ctx.fillStyle = "rgba(148, 163, 184, 0.9)";
        ctx.fillText(`g:${gStr} rhs:${rhsStr}`, sp.x, sp.y + baseRadius + (10 * zoom));

        ctx.restore();
    }

    function drawPathParticles() {
        if (!planningResult || !planningResult.statePath || planningResult.statePath.length < 2) return;
        particleOffset = (particleOffset + 0.015) % 1.0;

        ctx.save();
        ctx.fillStyle = "#ffffff";
        ctx.shadowColor = "#38bdf8";
        ctx.shadowBlur = 10;

        for (let i = 0; i + 1 < planningResult.statePath.length; i++) {
            const uId = planningResult.statePath[i];
            const vId = planningResult.statePath[i + 1];
            const u = currentProblem.states.find(s => s.id === uId);
            const v = currentProblem.states.find(s => s.id === vId);
            if (u && v) {
                const p1 = worldToScreen(u.embedding[0], u.embedding[1]);
                const p2 = worldToScreen(v.embedding[0], v.embedding[1]);
                const px = p1.x + (p2.x - p1.x) * particleOffset;
                const py = p1.y + (p2.y - p1.y) * particleOffset;

                ctx.beginPath();
                ctx.arc(px, py, 4 * zoom, 0, Math.PI * 2);
                ctx.fill();
            }
        }
        ctx.restore();
    }

    // --- Telemetry & Dashboard Updater ---
    function updateTelemetry() {
        if (!planningResult) {
            statStatus.textContent = "IDLE";
            statStatus.className = "stat-value";
            return;
        }

        if (planningResult.success) {
            statStatus.textContent = "OPTIMAL PATH";
            statStatus.className = "stat-value highlight-green";
            statCost.textContent = planningResult.totalCost.toFixed(2);
            
            if (planningResult.safetyScore < INF) {
                statSafety.textContent = planningResult.safetyScore.toFixed(2);
            } else {
                statSafety.textContent = "+INF (Safe)";
            }

            statReliability.textContent = (planningResult.cumulativeReliability * 100).toFixed(1) + "%";
            statScore.textContent = planningResult.objectiveScore.toFixed(1);
            statExplored.textContent = planningResult.exploredStates.toString();
            statTime.textContent = planningResult.planningTimeMicroseconds.toFixed(1) + " μs";
            pathText.textContent = planningResult.statePath.map(id => {
                const s = currentProblem.states.find(st => st.id === id);
                return s ? (s.label || `S${id}`) : `S${id}`;
            }).join(" ➔ ");
        } else {
            statStatus.textContent = "BLOCKED / NO PATH";
            statStatus.className = "stat-value highlight-rose";
            statCost.textContent = "N/A";
            statSafety.textContent = "0.00";
            statReliability.textContent = "0.0%";
            statScore.textContent = "0.0";
            pathText.textContent = "No safe collision-free path exists to goal!";
        }

        // Render Priority Queue Table
        updateQueueTable();
    }

    function updateQueueTable() {
        queueTableBody.innerHTML = "";
        const queueItems = planner.queue.slice(0, 10); // top 10

        if (queueItems.length === 0) {
            const tr = document.createElement("tr");
            tr.innerHTML = `<td colspan="4" style="text-align:center; color:var(--text-muted);">Queue Empty (Search Consistent)</td>`;
            queueTableBody.appendChild(tr);
            return;
        }

        queueItems.forEach(item => {
            const s = currentProblem.states.find(st => st.id === item.id);
            const gVal = planner.getG(item.id);
            const rhsVal = planner.getRhs(item.id);
            let statusTag = "";

            if (gVal > rhsVal) {
                statusTag = `<span class="status-tag status-over">Overconsistent</span>`;
            } else {
                statusTag = `<span class="status-tag status-under">Underconsistent</span>`;
            }

            const tr = document.createElement("tr");
            tr.innerHTML = `
                <td><strong>${s ? (s.label || `S${item.id}`) : item.id}</strong></td>
                <td>[${item.key.k1.toFixed(1)}, ${item.key.k2.toFixed(1)}]</td>
                <td>g:${gVal >= INF ? '∞' : gVal.toFixed(1)} / rhs:${rhsVal >= INF ? '∞' : rhsVal.toFixed(1)}</td>
                <td>${statusTag}</td>
            `;
            queueTableBody.appendChild(tr);
        });
    }

    // --- Interactive Tools & Event Handlers ---
    document.querySelectorAll(".tool-btn").forEach(btn => {
        btn.addEventListener("click", () => {
            document.querySelectorAll(".tool-btn").forEach(b => b.classList.remove("active"));
            btn.classList.add("active");
            activeTool = btn.dataset.tool;
            transitionFromId = null;
            selectedStateId = null;
        });
    });

    canvas.addEventListener("mousedown", (e) => {
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        const worldPos = screenToWorld(mouseX, mouseY);

        if (e.button === 1 || e.shiftKey) {
            // Pan
            isPanning = true;
            panStartX = mouseX - panX;
            panStartY = mouseY - panY;
            return;
        }

        const clickedState = findStateAt(worldPos.x, worldPos.y);

        if (activeTool === "select") {
            if (clickedState) {
                isDragging = true;
                dragTarget = clickedState;
                selectedStateId = clickedState.id;
            } else {
                selectedStateId = null;
            }
        } else if (activeTool === "addState") {
            const nextId = currentProblem.states.length > 0 
                ? Math.max(...currentProblem.states.map(s => s.id)) + 1 
                : 1;
            const newState = {
                id: nextId,
                embedding: [Math.round(worldPos.x), Math.round(worldPos.y)],
                label: `S${nextId}`
            };
            currentProblem.states.push(newState);
            planner.loadProblem(currentProblem);
            planningResult = planner.plan(currentProblem);
            updateTelemetry();
        } else if (activeTool === "addTransition") {
            if (clickedState) {
                if (transitionFromId === null) {
                    transitionFromId = clickedState.id;
                } else if (transitionFromId !== clickedState.id) {
                    const fromState = currentProblem.states.find(s => s.id === transitionFromId);
                    const dist = GeometryJS.euclidean(fromState.embedding, clickedState.embedding) / 100.0;
                    const nextTId = currentProblem.transitions.length > 0 
                        ? Math.max(...currentProblem.transitions.map(t => t.id)) + 1 
                        : 1;
                    const newTrans = {
                        id: nextTId,
                        from: transitionFromId,
                        to: clickedState.id,
                        cost: Math.max(1.0, parseFloat(dist.toFixed(1))),
                        safety: 1.0,
                        reliability: 0.98,
                        available: true
                    };
                    planner.addTransition(newTrans);
                    planningResult = planner.replanIncremental();
                    transitionFromId = null;
                    updateTelemetry();
                }
            }
        } else if (activeTool === "toggleBad") {
            if (clickedState) {
                const isBad = currentProblem.badStates.includes(clickedState.id);
                if (isBad) {
                    currentProblem.badStates = currentProblem.badStates.filter(id => id !== clickedState.id);
                    planner.setBadState(clickedState.id, false);
                } else {
                    currentProblem.badStates.push(clickedState.id);
                    planner.setBadState(clickedState.id, true);
                }
                planningResult = planner.replanIncremental();
                updateTelemetry();
            }
        } else if (activeTool === "toggleEdge") {
            const clickedEdge = findEdgeAt(worldPos.x, worldPos.y);
            if (clickedEdge) {
                const newAvail = !clickedEdge.available;
                clickedEdge.available = newAvail;
                planner.updateEdgeAvailability(clickedEdge.id, newAvail);
                planningResult = planner.replanIncremental();
                updateTelemetry();
            }
        } else if (activeTool === "setStart") {
            if (clickedState) {
                currentProblem.initialState = clickedState.id;
                planner.loadProblem(currentProblem);
                planningResult = planner.plan(currentProblem);
                updateTelemetry();
            }
        } else if (activeTool === "setGoal") {
            if (clickedState) {
                currentProblem.goalState = clickedState.id;
                planner.updateGoal(clickedState.id);
                planningResult = planner.replanIncremental();
                updateTelemetry();
            }
        }

        draw();
    });

    canvas.addEventListener("mousemove", (e) => {
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        const worldPos = screenToWorld(mouseX, mouseY);

        if (isPanning) {
            panX = mouseX - panStartX;
            panY = mouseY - panStartY;
            draw();
            return;
        }

        if (isDragging && dragTarget) {
            dragTarget.embedding = [Math.round(worldPos.x), Math.round(worldPos.y)];
            // When dragged, update heuristic calibration and replan
            planner.calibrateHeuristic();
            planningResult = planner.replanIncremental();
            updateTelemetry();
            draw();
            return;
        }

        const stateUnderMouse = findStateAt(worldPos.x, worldPos.y);
        const edgeUnderMouse = findEdgeAt(worldPos.x, worldPos.y);
        
        if (stateUnderMouse) {
            hoveredStateId = stateUnderMouse.id;
            hoveredEdgeId = null;
        } else if (edgeUnderMouse) {
            hoveredEdgeId = edgeUnderMouse.id;
            hoveredStateId = null;
        } else {
            hoveredStateId = null;
            hoveredEdgeId = null;
        }

        draw();
    });

    window.addEventListener("mouseup", () => {
        isDragging = false;
        dragTarget = null;
        isPanning = false;
    });

    canvas.addEventListener("wheel", (e) => {
        e.preventDefault();
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        const worldBefore = screenToWorld(mouseX, mouseY);

        const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
        zoom = Math.min(Math.max(zoom * zoomFactor, 0.25), 3.5);

        panX = mouseX - worldBefore.x * zoom;
        panY = mouseY - worldBefore.y * zoom;
        draw();
    });

    // --- Action Button Handlers ---
    btnPlanCold.addEventListener("click", () => {
        syncWeightsToProblem();
        planner.loadProblem(currentProblem);
        planningResult = planner.plan(currentProblem);
        updateTelemetry();
        draw();
    });

    btnReplanInc.addEventListener("click", () => {
        syncWeightsToProblem();
        planningResult = planner.replanIncremental();
        updateTelemetry();
        draw();
    });

    btnStep.addEventListener("click", () => {
        const stepRes = planner.stepSingle();
        planningResult = planner.extractResult();
        updateTelemetry();
        draw();
    });

    let isPlaying = false;
    btnPlay.addEventListener("click", () => {
        if (isPlaying) {
            clearInterval(animationStepInterval);
            isPlaying = false;
            btnPlay.textContent = "▶ Auto Step";
        } else {
            isPlaying = true;
            btnPlay.textContent = "⏸ Pause";
            animationStepInterval = setInterval(() => {
                const stepRes = planner.stepSingle();
                planningResult = planner.extractResult();
                updateTelemetry();
                draw();
                if (stepRes.done) {
                    clearInterval(animationStepInterval);
                    isPlaying = false;
                    btnPlay.textContent = "▶ Auto Step";
                }
            }, 250);
        }
    });

    btnReset.addEventListener("click", () => {
        loadScenario(scenarioSelect.value);
    });

    btnClear.addEventListener("click", () => {
        currentProblem = {
            name: "Custom Empty Canvas",
            description: "Click to add states and transitions.",
            initialState: 1,
            goalState: 2,
            badStates: [],
            states: [
                { id: 1, embedding: [200, 300], label: "Start" },
                { id: 2, embedding: [600, 300], label: "Goal" }
            ],
            transitions: []
        };
        planner.loadProblem(currentProblem);
        planningResult = planner.plan(currentProblem);
        updateTelemetry();
        draw();
    });

    // Slider Event Listeners
    [sliderAlpha, sliderBeta, sliderGamma, sliderDelta, sliderSafetyMargin].forEach(slider => {
        slider.addEventListener("input", () => {
            valAlpha.textContent = sliderAlpha.value;
            valBeta.textContent = sliderBeta.value;
            valGamma.textContent = sliderGamma.value;
            valDelta.textContent = sliderDelta.value;
            valSafetyMargin.textContent = sliderSafetyMargin.value;
            
            syncWeightsToProblem();
            planningResult = planner.replanIncremental();
            updateTelemetry();
            draw();
        });
    });

    scenarioSelect.addEventListener("change", (e) => {
        loadScenario(e.target.value);
    });

    // JSON Export & Import
    btnExportJson.addEventListener("click", () => {
        const jsonStr = JSON.stringify(currentProblem, null, 2);
        const blob = new Blob([jsonStr], { type: "application/json" });
        const url = URL.createObjectURL(blob);
        const a = document.createElement("a");
        a.href = url;
        a.download = `${scenarioSelect.value || "problem"}.json`;
        a.click();
        URL.revokeObjectURL(url);
    });

    fileImportInput.addEventListener("change", (e) => {
        const file = e.target.files[0];
        if (!file) return;
        const reader = new FileReader();
        reader.onload = (evt) => {
            try {
                const parsed = JSON.parse(evt.target.result);
                currentProblem = parsed;
                planner.loadProblem(currentProblem);
                planningResult = planner.plan(currentProblem);
                centerGraphInView();
                updateTelemetry();
                draw();
            } catch (err) {
                alert("Invalid JSON format: " + err.message);
            }
        };
        reader.readAsText(file);
    });

    // Continuous Animation Loop for Particles & Pulses
    function renderLoop() {
        if (planningResult && planningResult.success) {
            draw();
        }
        requestAnimationFrame(renderLoop);
    }

    // Initialize Default Scenario (Test Case 2: Bad State Avoidance)
    resizeCanvas();
    loadScenario("tc2");
    requestAnimationFrame(renderLoop);
});
