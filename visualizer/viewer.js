(function () {
  const VISUAL_AGENT_SIZE = 1.0; // requested fixed visual size: 1 grid unit

  const ndjsonInput = document.getElementById("ndjsonFile");
  const playPauseBtn = document.getElementById("playPause");
  const resetViewBtn = document.getElementById("resetView");
  const speedSelect = document.getElementById("speed");
  const tickSlider = document.getElementById("tickSlider");
  const tickLabel = document.getElementById("tickLabel");
  const status = document.getElementById("status");
  const bgCanvas = document.getElementById("bgCanvas");
  const glCanvas = document.getElementById("glCanvas");
  const viewport = document.getElementById("viewport");

  const state = {
    meta: null,
    ticks: [],
    currentTickIdx: 0,
    playing: false,
    speed: 1,
    accumMs: 0,
    lastFrameTs: 0,
    gl: null,
    program: null,
    posBuffer: null,
    dotPosBuffer: null,
    delayBuffer: null,
    aPos: -1,
    aDelay: -1,
    uViewMin: null,
    uViewSize: null,
    uPointSize: null,
    uColorNormal: null,
    uColorDelay: null,
    viewMinX: 0,
    viewMinY: 0,
    viewW: 1,
    viewH: 1,
    drag: null,
    mapCanvas: null
  };

  function setStatus(text) {
    status.textContent = text;
  }

  function setControlsEnabled(enabled) {
    playPauseBtn.disabled = !enabled;
    resetViewBtn.disabled = !enabled;
    speedSelect.disabled = !enabled;
    tickSlider.disabled = !enabled;
  }

  function parseNdjson(rawText) {
    const lines = rawText.split(/\r?\n/);
    let meta = null;
    const ticks = [];

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i].trim();
      if (!line) {
        continue;
      }
      const obj = JSON.parse(line);
      if (obj.type === "meta") {
        meta = obj;
        continue;
      }
      if (obj.type === "tick") {
        const loc = Int32Array.from(obj.loc || []);
        const x = obj.x ? Float32Array.from(obj.x) : null;
        const y = obj.y ? Float32Array.from(obj.y) : null;
        const ori = Uint8Array.from(obj.ori || []);
        const mv = obj.mv ? Int8Array.from(obj.mv) : null;
        const cnt = obj.cnt ? Int16Array.from(obj.cnt) : null;
        const maxCnt = obj.maxCnt ? Int16Array.from(obj.maxCnt) : null;
        const act = obj.act ? Int8Array.from(obj.act) : null;
        const rdir = obj.rdir ? Int8Array.from(obj.rdir) : null;
        const delay = Uint8Array.from(obj.delay || []);
        ticks.push({
          t: obj.t,
          loc,
          x,
          y,
          ori,
          mv,
          cnt,
          maxCnt,
          act,
          rdir,
          delay,
          pos: null,
          dotPos: null,
          delayFloat: null
        });
      }
    }

    if (!meta) {
      throw new Error("Missing meta record in NDJSON");
    }
    if (ticks.length === 0) {
      throw new Error("No tick records found");
    }
    return { meta, ticks };
  }

  function buildMapCanvas(meta) {
    const canvas = document.createElement("canvas");
    canvas.width = meta.cols;
    canvas.height = meta.rows;
    const ctx = canvas.getContext("2d");
    const img = ctx.createImageData(meta.cols, meta.rows);

    for (let i = 0; i < img.data.length; i += 4) {
      img.data[i] = 247;
      img.data[i + 1] = 250;
      img.data[i + 2] = 252;
      img.data[i + 3] = 255;
    }

    const obstacles = meta.obstacles || [];
    for (let i = 0; i < obstacles.length; i++) {
      const loc = obstacles[i];
      const row = Math.floor(loc / meta.cols);
      const col = loc % meta.cols;
      const p = (row * meta.cols + col) * 4;
      img.data[p] = 182;
      img.data[p + 1] = 193;
      img.data[p + 2] = 203;
      img.data[p + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
    return canvas;
  }

  function ensureTickBuffers(tick) {
    if (tick.pos && tick.delayFloat) {
      return;
    }
    const n = tick.loc.length;
    const pos = new Float32Array(n * 2);
    const dotPos = new Float32Array(n * 2);
    const delay = new Float32Array(n);
    const markerRadius = VISUAL_AGENT_SIZE * 0.5;
    const dotOffset = markerRadius * 0.6;
    const quarterTurn = Math.PI * 0.5;

    function baseAngleFromOrientation(ori) {
      if (ori === 0) return 0; // east
      if (ori === 1) return quarterTurn; // south
      if (ori === 2) return Math.PI; // west
      return -quarterTurn; // north
    }

    for (let i = 0; i < n; i++) {
      const loc = tick.loc[i];
      const row = Math.floor(loc / state.meta.cols);
      const col = loc % state.meta.cols;
      const baseX = tick.x ? tick.x[i] : col;
      const baseY = tick.y ? tick.y[i] : row;
      const centerX = baseX + markerRadius;
      const centerY = baseY + markerRadius;
      pos[i * 2] = centerX;
      pos[i * 2 + 1] = centerY;
      const ori = tick.ori[i] || 0;
      let angle = baseAngleFromOrientation(ori);
      const moveType = tick.mv ? tick.mv[i] : 2;
      const cnt = tick.cnt ? tick.cnt[i] : 0;
      const maxCnt = tick.maxCnt ? tick.maxCnt[i] : 0;
      const rdir = tick.rdir ? tick.rdir[i] : 0;
      if (moveType === 1 && maxCnt > 0 && cnt > 0 && rdir !== 0) {
        const frac = cnt / maxCnt;
        angle += rdir * frac * quarterTurn;
      }
      const dx = Math.cos(angle) * dotOffset;
      const dy = Math.sin(angle) * dotOffset;
      dotPos[i * 2] = centerX + dx;
      dotPos[i * 2 + 1] = centerY + dy;
      delay[i] = tick.delay[i] ? 1 : 0;
    }
    tick.pos = pos;
    tick.dotPos = dotPos;
    tick.delayFloat = delay;
  }

  function createShader(gl, type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      const msg = gl.getShaderInfoLog(shader);
      gl.deleteShader(shader);
      throw new Error(msg || "Shader compile failed");
    }
    return shader;
  }

  function initGl() {
    const gl = glCanvas.getContext("webgl2", { antialias: false, alpha: true });
    if (!gl) {
      throw new Error("WebGL2 not available in this browser");
    }

    const vs = `#version 300 es
      in vec2 a_pos;
      in float a_delay;
      uniform vec2 u_view_min;
      uniform vec2 u_view_size;
      uniform float u_point_size;
      out float v_delay;
      void main() {
        float nx = ((a_pos.x - u_view_min.x) / u_view_size.x) * 2.0 - 1.0;
        float ny = 1.0 - ((a_pos.y - u_view_min.y) / u_view_size.y) * 2.0;
        gl_Position = vec4(nx, ny, 0.0, 1.0);
        gl_PointSize = u_point_size;
        v_delay = a_delay;
      }
    `;
    const fs = `#version 300 es
      precision mediump float;
      in float v_delay;
      uniform vec3 u_color_normal;
      uniform vec3 u_color_delay;
      out vec4 out_color;
      void main() {
        vec2 c = gl_PointCoord * 2.0 - 1.0;
        float d = dot(c, c);
        if (d > 1.0) {
          discard;
        }
        vec3 color = mix(u_color_normal, u_color_delay, clamp(v_delay, 0.0, 1.0));
        out_color = vec4(color, 1.0);
      }
    `;

    const program = gl.createProgram();
    gl.attachShader(program, createShader(gl, gl.VERTEX_SHADER, vs));
    gl.attachShader(program, createShader(gl, gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(gl.getProgramInfoLog(program) || "Program link failed");
    }

    const posBuffer = gl.createBuffer();
    const dotPosBuffer = gl.createBuffer();
    const delayBuffer = gl.createBuffer();

    state.gl = gl;
    state.program = program;
    state.posBuffer = posBuffer;
    state.dotPosBuffer = dotPosBuffer;
    state.delayBuffer = delayBuffer;
    state.aPos = gl.getAttribLocation(program, "a_pos");
    state.aDelay = gl.getAttribLocation(program, "a_delay");
    state.uViewMin = gl.getUniformLocation(program, "u_view_min");
    state.uViewSize = gl.getUniformLocation(program, "u_view_size");
    state.uPointSize = gl.getUniformLocation(program, "u_point_size");
    state.uColorNormal = gl.getUniformLocation(program, "u_color_normal");
    state.uColorDelay = gl.getUniformLocation(program, "u_color_delay");
  }

  function resetView() {
    const cols = state.meta.cols;
    const rows = state.meta.rows;
    const w = viewport.clientWidth || 1;
    const h = viewport.clientHeight || 1;
    const viewportAspect = w / h;
    const mapAspect = cols / rows;

    if (viewportAspect >= mapAspect) {
      state.viewH = rows;
      state.viewW = rows * viewportAspect;
      state.viewMinX = -(state.viewW - cols) / 2;
      state.viewMinY = 0;
    } else {
      state.viewW = cols;
      state.viewH = cols / viewportAspect;
      state.viewMinX = 0;
      state.viewMinY = -(state.viewH - rows) / 2;
    }
  }

  function resizeCanvases() {
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.floor(viewport.clientWidth * dpr));
    const h = Math.max(1, Math.floor(viewport.clientHeight * dpr));

    if (bgCanvas.width !== w || bgCanvas.height !== h) {
      bgCanvas.width = w;
      bgCanvas.height = h;
    }
    if (glCanvas.width !== w || glCanvas.height !== h) {
      glCanvas.width = w;
      glCanvas.height = h;
    }
  }

  function drawBackground() {
    const ctx = bgCanvas.getContext("2d");
    ctx.imageSmoothingEnabled = false;
    ctx.fillStyle = "#f8fafc";
    ctx.fillRect(0, 0, bgCanvas.width, bgCanvas.height);

    if (!state.mapCanvas || !state.meta) {
      return;
    }

    const sx = state.viewMinX;
    const sy = state.viewMinY;
    const sw = state.viewW;
    const sh = state.viewH;

    ctx.drawImage(
      state.mapCanvas,
      sx,
      sy,
      sw,
      sh,
      0,
      0,
      bgCanvas.width,
      bgCanvas.height
    );

    const mapLeft = ((0 - state.viewMinX) / state.viewW) * bgCanvas.width;
    const mapTop = ((0 - state.viewMinY) / state.viewH) * bgCanvas.height;
    const mapRight = ((state.meta.cols - state.viewMinX) / state.viewW) * bgCanvas.width;
    const mapBottom = ((state.meta.rows - state.viewMinY) / state.viewH) * bgCanvas.height;

    // Draw grid lines at cell boundaries inside the map only.
    const pxPerCellX = bgCanvas.width / state.viewW;
    const pxPerCellY = bgCanvas.height / state.viewH;
    if (pxPerCellX >= 3 || pxPerCellY >= 3) {
      const xStart = Math.max(0, Math.floor(state.viewMinX));
      const xEnd = Math.min(state.meta.cols, Math.ceil(state.viewMinX + state.viewW));
      const yStart = Math.max(0, Math.floor(state.viewMinY));
      const yEnd = Math.min(state.meta.rows, Math.ceil(state.viewMinY + state.viewH));
      ctx.save();
      ctx.beginPath();
      ctx.rect(mapLeft, mapTop, mapRight - mapLeft, mapBottom - mapTop);
      ctx.clip();
      ctx.beginPath();
      ctx.strokeStyle = "rgba(30,41,59,0.18)";
      ctx.lineWidth = 1;
      for (let gx = xStart; gx <= xEnd; gx++) {
        const px = ((gx - state.viewMinX) / state.viewW) * bgCanvas.width;
        ctx.moveTo(px, 0);
        ctx.lineTo(px, bgCanvas.height);
      }
      for (let gy = yStart; gy <= yEnd; gy++) {
        const py = ((gy - state.viewMinY) / state.viewH) * bgCanvas.height;
        ctx.moveTo(0, py);
        ctx.lineTo(bgCanvas.width, py);
      }
      ctx.stroke();
      ctx.restore();
    }

    // Draw map border so map extent is clear after pan/zoom.
    ctx.strokeStyle = "rgba(15,23,42,0.85)";
    ctx.lineWidth = 2;
    ctx.strokeRect(mapLeft, mapTop, mapRight - mapLeft, mapBottom - mapTop);
  }

  function uploadTickToGpu(tick) {
    const gl = state.gl;
    ensureTickBuffers(tick);

    gl.bindBuffer(gl.ARRAY_BUFFER, state.posBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, tick.pos, gl.DYNAMIC_DRAW);

    gl.bindBuffer(gl.ARRAY_BUFFER, state.dotPosBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, tick.dotPos, gl.DYNAMIC_DRAW);

    gl.bindBuffer(gl.ARRAY_BUFFER, state.delayBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, tick.delayFloat, gl.DYNAMIC_DRAW);
  }

  function renderGl() {
    if (!state.gl || !state.meta || state.ticks.length === 0) {
      return;
    }
    const gl = state.gl;
    const tick = state.ticks[state.currentTickIdx];
    uploadTickToGpu(tick);

    gl.viewport(0, 0, glCanvas.width, glCanvas.height);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);

    gl.useProgram(state.program);
    gl.uniform2f(state.uViewMin, state.viewMinX, state.viewMinY);
    gl.uniform2f(state.uViewSize, state.viewW, state.viewH);
    const pxPerCell = Math.min(glCanvas.width / state.viewW, glCanvas.height / state.viewH);
    const basePointSize = Math.max(1.0, VISUAL_AGENT_SIZE * pxPerCell);
    gl.uniform1f(state.uPointSize, basePointSize);
    gl.uniform3f(state.uColorNormal, 0.066, 0.647, 0.914);
    gl.uniform3f(state.uColorDelay, 0.875, 0.204, 0.204);

    gl.bindBuffer(gl.ARRAY_BUFFER, state.posBuffer);
    gl.enableVertexAttribArray(state.aPos);
    gl.vertexAttribPointer(state.aPos, 2, gl.FLOAT, false, 0, 0);

    gl.bindBuffer(gl.ARRAY_BUFFER, state.delayBuffer);
    gl.enableVertexAttribArray(state.aDelay);
    gl.vertexAttribPointer(state.aDelay, 1, gl.FLOAT, false, 0, 0);

    gl.drawArrays(gl.POINTS, 0, tick.loc.length);

    // Orientation indicator dot
    gl.uniform1f(state.uPointSize, Math.max(1.0, basePointSize * 0.28));
    gl.uniform3f(state.uColorNormal, 0.07, 0.10, 0.15);
    gl.uniform3f(state.uColorDelay, 0.07, 0.10, 0.15);
    gl.bindBuffer(gl.ARRAY_BUFFER, state.dotPosBuffer);
    gl.enableVertexAttribArray(state.aPos);
    gl.vertexAttribPointer(state.aPos, 2, gl.FLOAT, false, 0, 0);
    gl.drawArrays(gl.POINTS, 0, tick.loc.length);
  }

  function render() {
    drawBackground();
    renderGl();
    updateTickLabel();
  }

  function frameIntervalMs() {
    return (state.meta.tickMs || 100) * (state.meta.tickStride || 1) / state.speed;
  }

  function updateTickLabel() {
    if (!state.meta || state.ticks.length === 0) {
      tickLabel.textContent = "tick -";
      return;
    }
    const tick = state.ticks[state.currentTickIdx];
    tickLabel.textContent = `tick ${tick.t} (${state.currentTickIdx + 1}/${state.ticks.length})`;
  }

  function setTickIndex(newIdx) {
    const clamped = Math.max(0, Math.min(state.ticks.length - 1, newIdx));
    state.currentTickIdx = clamped;
    tickSlider.value = String(clamped);
    render();
  }

  function animate(ts) {
    if (state.playing && state.ticks.length > 0) {
      if (state.lastFrameTs > 0) {
        state.accumMs += (ts - state.lastFrameTs);
      }
      const stepMs = frameIntervalMs();
      while (state.accumMs >= stepMs && state.currentTickIdx < state.ticks.length - 1) {
        state.currentTickIdx += 1;
        state.accumMs -= stepMs;
      }
      if (state.currentTickIdx >= state.ticks.length - 1) {
        state.playing = false;
        playPauseBtn.textContent = "Play";
      }
      tickSlider.value = String(state.currentTickIdx);
      render();
    }
    state.lastFrameTs = ts;
    window.requestAnimationFrame(animate);
  }

  function screenToWorld(clientX, clientY) {
    const rect = viewport.getBoundingClientRect();
    const px = clientX - rect.left;
    const py = clientY - rect.top;
    const worldX = state.viewMinX + (px / rect.width) * state.viewW;
    const worldY = state.viewMinY + (py / rect.height) * state.viewH;
    return { x: worldX, y: worldY };
  }

  function clampView() {
    if (!state.meta) {
      return;
    }
    const minW = 8;
    const minH = 8;
    const maxW = state.meta.cols * 4;
    const maxH = state.meta.rows * 4;
    state.viewW = Math.max(minW, Math.min(maxW, state.viewW));
    state.viewH = Math.max(minH, Math.min(maxH, state.viewH));
  }

  function installInteractions() {
    viewport.addEventListener("wheel", (ev) => {
      if (!state.meta) {
        return;
      }
      ev.preventDefault();
      const anchor = screenToWorld(ev.clientX, ev.clientY);
      const zoomFactor = Math.exp(ev.deltaY * 0.0015);
      state.viewW *= zoomFactor;
      state.viewH *= zoomFactor;
      clampView();
      const rect = viewport.getBoundingClientRect();
      const px = ev.clientX - rect.left;
      const py = ev.clientY - rect.top;
      state.viewMinX = anchor.x - (px / rect.width) * state.viewW;
      state.viewMinY = anchor.y - (py / rect.height) * state.viewH;
      render();
    }, { passive: false });

    viewport.addEventListener("mousedown", (ev) => {
      if (!state.meta) {
        return;
      }
      state.drag = {
        x: ev.clientX,
        y: ev.clientY,
        minX: state.viewMinX,
        minY: state.viewMinY
      };
    });

    window.addEventListener("mousemove", (ev) => {
      if (!state.drag) {
        return;
      }
      const rect = viewport.getBoundingClientRect();
      const dx = ev.clientX - state.drag.x;
      const dy = ev.clientY - state.drag.y;
      state.viewMinX = state.drag.minX - (dx / rect.width) * state.viewW;
      state.viewMinY = state.drag.minY - (dy / rect.height) * state.viewH;
      render();
    });

    window.addEventListener("mouseup", () => {
      state.drag = null;
    });
  }

  async function loadFromFile(file) {
    setStatus("Loading and parsing NDJSON...");
    const raw = await file.text();
    const parsed = parseNdjson(raw);
    state.meta = parsed.meta;
    state.ticks = parsed.ticks;
    state.currentTickIdx = 0;
    state.playing = false;
    state.accumMs = 0;
    state.lastFrameTs = 0;
    state.mapCanvas = buildMapCanvas(state.meta);
    tickSlider.min = "0";
    tickSlider.max = String(Math.max(0, state.ticks.length - 1));
    tickSlider.value = "0";

    resetView();
    resizeCanvases();
    render();

    setControlsEnabled(true);
    playPauseBtn.textContent = "Play";
    setStatus(
      `Loaded ${state.ticks.length} ticks, ${state.meta.teamSize} agents, ${state.meta.rows}x${state.meta.cols} map. Red robots are delayed. Drag to pan, wheel to zoom.`
    );
  }

  ndjsonInput.addEventListener("change", async (ev) => {
    const file = ev.target.files && ev.target.files[0];
    if (!file) {
      return;
    }
    try {
      await loadFromFile(file);
    } catch (err) {
      setStatus(`Failed to load file: ${err.message}`);
      setControlsEnabled(false);
      throw err;
    }
  });

  playPauseBtn.addEventListener("click", () => {
    state.playing = !state.playing;
    playPauseBtn.textContent = state.playing ? "Pause" : "Play";
  });

  resetViewBtn.addEventListener("click", () => {
    if (!state.meta) {
      return;
    }
    resetView();
    resizeCanvases();
    render();
  });

  speedSelect.addEventListener("change", () => {
    state.speed = Number(speedSelect.value || "1");
  });

  tickSlider.addEventListener("input", () => {
    const idx = Number(tickSlider.value || "0");
    setTickIndex(idx);
  });

  window.addEventListener("resize", () => {
    if (!state.meta) {
      return;
    }
    resizeCanvases();
    render();
  });

  setControlsEnabled(false);
  installInteractions();
  initGl();
  resizeCanvases();
  window.requestAnimationFrame(animate);
})();
