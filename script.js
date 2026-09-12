import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { rk4Step, totalEnergy } from "./physics.js";
import {
  sphericalStateFromAngles,
  sphericalAnglesFromState,
  sphericalStep,
  sphericalEnergy,
} from "./spherical.js";

const DT = 0.002;
const MAX_SUBSTEPS = 100;
const TRAIL_CAPACITY = 2000;
const TRAIL_FADE_SECONDS = 5;

const PALETTE_A = {
  rod: 0x4fc3f7,
  bob1: 0xff5722,
  bob2: 0x2196f3,
  trail1: 0xff5722,
  trail2: 0x2196f3,
};

const PALETTE_B = {
  rod: 0x81c784,
  bob1: 0x4caf50,
  bob2: 0x9c27b0,
  trail1: 0x4caf50,
  trail2: 0x9c27b0,
};

// 混沌对比:B 摆参数与 A 完全相同,仅一个初值相差 δ
const compare = {
  enabled: false,
  field: "theta1",

  get delta() {
    const raw = parseFloat(document.getElementById("compare-delta").value);
    return Number.isFinite(raw) ? raw : 0;
  },

  // 角度差(θ/φ)输入为度,ω 差输入为 rad/s
  deltaRadians() {
    const raw = this.delta;
    return this.field.startsWith("omega") ? raw : (raw * Math.PI) / 180;
  },

  resync() {
    if (!pendulumB) return;
    if (physicsMode === "planar") {
      pendulumB.state = { ...pendulumA.state };
      pendulumB.state[this.field] += this.deltaRadians();
    } else {
      // 球面:反解 A 的角度初值,施加 δ 后重建笛卡尔状态
      const angles = sphericalAnglesFromState(pendulumA.state, params);
      angles[this.field] += this.deltaRadians();
      pendulumB.state = sphericalStateFromAngles(angles, params);
    }
    pendulumB.clearTrails();
    pendulumB.updateVisuals();
  },

  enable() {
    pendulumB = new Pendulum(scene, PALETTE_B, physicsMode);
    pendulumB.updateVisuals();
    this.resync();
    document.getElementById("compare-readout").style.display = "block";
  },

  disable() {
    if (!pendulumB) return;
    pendulumB.dispose(scene);
    pendulumB = null;
    document.getElementById("compare-readout").style.display = "none";
  },
};

function formatDelta(value) {
  return value < 1e-3 ? value.toExponential(2) : value.toFixed(4);
}

// ---------- 物理模式(平面/球面) ----------

const COMPARE_FIELDS = {
  planar: [
    ["theta1", "角度 θ₁"],
    ["theta2", "角度 θ₂"],
    ["omega1", "角速度 ω₁"],
    ["omega2", "角速度 ω₂"],
  ],
  spherical: [
    ["theta1", "角度 θ₁"],
    ["theta2", "角度 θ₂"],
    ["phi1", "方位角 φ₁"],
    ["phi2", "方位角 φ₂"],
    ["omega1", "角速度 ω₁"],
    ["omega2", "角速度 ω₂"],
    ["omegaphi1", "角速度 ωφ₁"],
    ["omegaphi2", "角速度 ωφ₂"],
  ],
};

function rebuildCompareFieldOptions() {
  const select = document.getElementById("compare-field");
  const options = COMPARE_FIELDS[physicsMode];
  select.innerHTML = options
    .map(([value, label]) => `<option value="${value}">${label}</option>`)
    .join("");
  if (!options.some(([value]) => value === compare.field)) {
    compare.field = options[0][0];
  }
  select.value = compare.field;
}

// 切换模式/初始化时(重)建两摆:销毁旧 three.js 对象,按滑条初值重建状态
function recreatePendulums() {
  isPlaying = false;
  simTime = 0;
  if (pendulumA) pendulumA.dispose(scene);
  if (pendulumB) {
    pendulumB.dispose(scene);
    pendulumB = null;
  }
  pendulumA = new Pendulum(scene, PALETTE_A, physicsMode);
  pendulumA.resetToInitial();
  pendulumA.updateVisuals();
  if (compare.enabled) {
    pendulumB = new Pendulum(scene, PALETTE_B, physicsMode);
    compare.resync();
  }
  updateDataDisplay();
}

// 摆参数(A/B 共用);theta/phi 单位弧度,omega 单位 rad/s
const params = {
  m1: 1.0,
  m2: 1.0,
  l1: 1.0,
  l2: 1.0,
  theta1: Math.PI / 2,
  theta2: Math.PI / 2,
  omega1: 0,
  omega2: 0,
  phi1: 0,
  phi2: 0,
  omegaphi1: 0,
  omegaphi2: 0,
};

let scene, camera, renderer, controls;
let pendulumA = null;
let pendulumB = null; // 混沌对比模式的 B 摆,下方 compare 区创建
let physicsMode = "planar"; // "planar" | "spherical"
let simTime = 0;
let isPlaying = false;
let lastFrameTime = null;

// ---------- 轨迹:环形缓冲 + 渲染线 ----------

class TrailRing {
  constructor(capacity) {
    this.capacity = capacity;
    this.x = new Float32Array(capacity);
    this.y = new Float32Array(capacity);
    this.z = new Float32Array(capacity);
    this.t = new Float64Array(capacity);
    this.head = 0;
    this.count = 0;
  }

  push(x, y, z, time) {
    const idx = (this.head + this.count) % this.capacity;
    if (this.count < this.capacity) {
      this.count++;
    } else {
      this.head = (this.head + 1) % this.capacity;
    }
    this.x[idx] = x;
    this.y[idx] = y;
    this.z[idx] = z;
    this.t[idx] = time;
  }

  trim(currentTime) {
    while (
      this.count > 0 &&
      currentTime - this.t[this.head] > TRAIL_FADE_SECONDS
    ) {
      this.head = (this.head + 1) % this.capacity;
      this.count--;
    }
  }

  clear() {
    this.head = 0;
    this.count = 0;
  }
}

class TrailLine {
  constructor(scene, color) {
    this.ring = new TrailRing(TRAIL_CAPACITY);
    this.positions = new Float32Array(TRAIL_CAPACITY * 3);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(this.positions, 3),
    );
    geometry.setDrawRange(0, 0);
    const material = new THREE.LineBasicMaterial({
      color,
      transparent: true,
      opacity: 0.7,
    });
    this.line = new THREE.Line(geometry, material);
    this.line.frustumCulled = false;
    scene.add(this.line);
  }

  // 把环形缓冲按“旧→新”顺序摊平进 geometry
  sync() {
    const { ring, positions } = this;
    for (let i = 0; i < ring.count; i++) {
      const src = (ring.head + i) % ring.capacity;
      positions[i * 3] = ring.x[src];
      positions[i * 3 + 1] = ring.y[src];
      positions[i * 3 + 2] = ring.z[src];
    }
    this.line.geometry.attributes.position.needsUpdate = true;
    this.line.geometry.setDrawRange(0, ring.count);
  }

  clear() {
    this.ring.clear();
    this.line.geometry.setDrawRange(0, 0);
  }

  dispose(scene) {
    scene.remove(this.line);
    this.line.geometry.dispose();
    this.line.material.dispose();
  }
}

// ---------- 摆体 ----------

function setLine(line, x0, y0, z0, x1, y1, z1) {
  const a = line.geometry.attributes.position.array;
  a[0] = x0;
  a[1] = y0;
  a[2] = z0;
  a[3] = x1;
  a[4] = y1;
  a[5] = z1;
  line.geometry.attributes.position.needsUpdate = true;
}

class Pendulum {
  constructor(scene, palette, mode) {
    this.mode = mode;
    this.palette = palette;
    this.state =
      mode === "planar"
        ? { theta1: 0, theta2: 0, omega1: 0, omega2: 0 }
        : sphericalStateFromAngles(
            {
              theta1: 0, phi1: 0, omega1: 0, omegaphi1: 0,
              theta2: 0, phi2: 0, omega2: 0, omegaphi2: 0,
            },
            params,
          );
    // 供 pushTrails 使用的末端坐标缓存
    this.x1 = 0;
    this.y1 = 0;
    this.z1 = 0;
    this.x2 = 0;
    this.y2 = 0;
    this.z2 = 0;

    this.rodMaterial = new THREE.LineBasicMaterial({ color: palette.rod });
    this.rod1 = this.makeRod(scene);
    this.rod2 = this.makeRod(scene);

    this.bob1 = new THREE.Mesh(
      new THREE.SphereGeometry(0.08, 16, 16),
      new THREE.MeshStandardMaterial({
        color: palette.bob1,
        metalness: 0.3,
        roughness: 0.4,
      }),
    );
    this.bob2 = new THREE.Mesh(
      new THREE.SphereGeometry(0.08, 16, 16),
      new THREE.MeshStandardMaterial({
        color: palette.bob2,
        metalness: 0.3,
        roughness: 0.4,
      }),
    );
    scene.add(this.bob1);
    scene.add(this.bob2);

    this.trail1 = new TrailLine(scene, palette.trail1);
    this.trail2 = new TrailLine(scene, palette.trail2);
  }

  makeRod(scene) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(new Float32Array(6), 3),
    );
    const rod = new THREE.Line(geometry, this.rodMaterial);
    rod.frustumCulled = false;
    scene.add(rod);
    return rod;
  }

  resetToInitial() {
    if (this.mode === "planar") {
      this.state.theta1 = params.theta1;
      this.state.theta2 = params.theta2;
      this.state.omega1 = params.omega1;
      this.state.omega2 = params.omega2;
    } else {
      this.state = sphericalStateFromAngles(
        {
          theta1: params.theta1,
          phi1: params.phi1,
          omega1: params.omega1,
          omegaphi1: params.omegaphi1,
          theta2: params.theta2,
          phi2: params.phi2,
          omega2: params.omega2,
          omegaphi2: params.omegaphi2,
        },
        params,
      );
    }
  }

  step() {
    if (this.mode === "planar") {
      rk4Step(this.state, params, DT);
    } else {
      sphericalStep(this.state, params, DT);
    }
  }

  isFiniteState() {
    if (this.mode === "planar") {
      return (
        Number.isFinite(this.state.theta1) &&
        Number.isFinite(this.state.theta2) &&
        Number.isFinite(this.state.omega1) &&
        Number.isFinite(this.state.omega2)
      );
    }
    const s = this.state;
    return [...s.p1, ...s.v1, ...s.p2, ...s.v2].every(Number.isFinite);
  }

  updateVisuals() {
    let x1, y1, z1, x2, y2, z2;
    if (this.mode === "planar") {
      x1 = params.l1 * Math.sin(this.state.theta1);
      y1 = -params.l1 * Math.cos(this.state.theta1);
      z1 = 0;
      x2 = x1 + params.l2 * Math.sin(this.state.theta2);
      y2 = y1 - params.l2 * Math.cos(this.state.theta2);
      z2 = 0;
    } else {
      [x1, y1, z1] = this.state.p1;
      [x2, y2, z2] = this.state.p2;
    }
    setLine(this.rod1, 0, 0, 0, x1, y1, z1);
    setLine(this.rod2, x1, y1, z1, x2, y2, z2);
    this.bob1.position.set(x1, y1, z1);
    this.bob2.position.set(x2, y2, z2);
    this.x1 = x1;
    this.y1 = y1;
    this.z1 = z1;
    this.x2 = x2;
    this.y2 = y2;
    this.z2 = z2;
  }

  pushTrails(simTime) {
    if (!document.getElementById("show-trail").checked) return;
    this.trail1.ring.push(this.x1, this.y1, this.z1, simTime);
    this.trail2.ring.push(this.x2, this.y2, this.z2, simTime);
    this.trail1.ring.trim(simTime);
    this.trail2.ring.trim(simTime);
    this.trail1.sync();
    this.trail2.sync();
  }

  clearTrails() {
    this.trail1.clear();
    this.trail2.clear();
  }

  dispose(scene) {
    scene.remove(this.rod1);
    scene.remove(this.rod2);
    scene.remove(this.bob1);
    scene.remove(this.bob2);
    this.rod1.geometry.dispose();
    this.rod2.geometry.dispose();
    this.rodMaterial.dispose();
    this.bob1.geometry.dispose();
    this.bob1.material.dispose();
    this.bob2.geometry.dispose();
    this.bob2.material.dispose();
    this.trail1.dispose(scene);
    this.trail2.dispose(scene);
  }
}

// ---------- 显示辅助 ----------

// 弧度 → (−180°, 180°]
function normalizeDeg(rad) {
  let deg = (rad * 180) / Math.PI;
  deg = (((deg + 180) % 360) + 360) % 360 - 180;
  if (deg === -180) deg = 180;
  return deg;
}

function updateDataDisplay() {
  let th1, th2, om1, om2, energy;
  if (physicsMode === "planar") {
    th1 = normalizeDeg(pendulumA.state.theta1);
    th2 = normalizeDeg(pendulumA.state.theta2);
    om1 = pendulumA.state.omega1;
    om2 = pendulumA.state.omega2;
    energy = totalEnergy(pendulumA.state, params);
  } else {
    const a = sphericalAnglesFromState(pendulumA.state, params);
    th1 = normalizeDeg(a.theta1);
    th2 = normalizeDeg(a.theta2);
    om1 = a.omega1;
    om2 = a.omega2;
    energy = sphericalEnergy(pendulumA.state, params);
  }
  document.getElementById("current-theta1").textContent = th1.toFixed(1);
  document.getElementById("current-theta2").textContent = th2.toFixed(1);
  document.getElementById("current-omega1").textContent = om1.toFixed(3);
  document.getElementById("current-omega2").textContent = om2.toFixed(3);
  document.getElementById("total-energy").textContent = energy.toFixed(3);
  document.getElementById("run-time").textContent = simTime.toFixed(2);

  if (pendulumB) {
    let d1, d2;
    if (physicsMode === "planar") {
      d1 =
        (Math.abs(pendulumA.state.theta1 - pendulumB.state.theta1) * 180) /
        Math.PI;
      d2 =
        (Math.abs(pendulumA.state.theta2 - pendulumB.state.theta2) * 180) /
        Math.PI;
    } else {
      const aa = sphericalAnglesFromState(pendulumA.state, params);
      const ab = sphericalAnglesFromState(pendulumB.state, params);
      d1 = (Math.abs(aa.theta1 - ab.theta1) * 180) / Math.PI;
      d2 = (Math.abs(aa.theta2 - ab.theta2) * 180) / Math.PI;
    }
    document.getElementById("delta-theta1").textContent = formatDelta(d1);
    document.getElementById("delta-theta2").textContent = formatDelta(d2);
  }
}

// ---------- UI 绑定 ----------

function bindSlider(id, valueId, onChange, format = (v) => v.toFixed(1)) {
  const input = document.getElementById(id);
  const label = document.getElementById(valueId);
  input.addEventListener("input", (e) => {
    const value = parseFloat(e.target.value);
    label.textContent = format(value);
    onChange(value);
  });
}

function clearAllTrails() {
  pendulumA.clearTrails();
  if (pendulumB) pendulumB.clearTrails();
}

// 初值滑条(θ/φ/ω/ωφ):仅暂停时可调;按滑条整体重建初始状态(两模式一致),
// B 摆(若启用)经 resync 自动跟随保持 δ
function onInitialConditionChange(field, value) {
  params[field] = value;
  if (isPlaying) return;
  pendulumA.resetToInitial();
  clearAllTrails();
  pendulumA.updateVisuals();
  if (compare.enabled) compare.resync();
  updateDataDisplay();
}

function onPlay() {
  isPlaying = true;
  hideWarning();
}

function onPause() {
  isPlaying = false;
}

function onReset() {
  isPlaying = false;
  simTime = 0;
  pendulumA.resetToInitial();
  clearAllTrails();
  pendulumA.updateVisuals();
  if (compare.enabled) compare.resync();
  updateDataDisplay();
}

function setupControls() {
  bindSlider("l1", "l1-value", (v) => {
    params.l1 = v;
    clearAllTrails();
  });
  bindSlider("l2", "l2-value", (v) => {
    params.l2 = v;
    clearAllTrails();
  });
  bindSlider("m1", "m1-value", (v) => {
    params.m1 = v;
    clearAllTrails();
    updateDataDisplay();
  });
  bindSlider("m2", "m2-value", (v) => {
    params.m2 = v;
    clearAllTrails();
    updateDataDisplay();
  });
  bindSlider(
    "theta1",
    "theta1-value",
    (v) => onInitialConditionChange("theta1", (v * Math.PI) / 180),
    (v) => `${v}`,
  );
  bindSlider(
    "theta2",
    "theta2-value",
    (v) => onInitialConditionChange("theta2", (v * Math.PI) / 180),
    (v) => `${v}`,
  );
  bindSlider("omega1", "omega1-value", (v) =>
    onInitialConditionChange("omega1", v),
  );
  bindSlider("omega2", "omega2-value", (v) =>
    onInitialConditionChange("omega2", v),
  );
  bindSlider(
    "phi1",
    "phi1-value",
    (v) => onInitialConditionChange("phi1", (v * Math.PI) / 180),
    (v) => `${v}`,
  );
  bindSlider(
    "phi2",
    "phi2-value",
    (v) => onInitialConditionChange("phi2", (v * Math.PI) / 180),
    (v) => `${v}`,
  );
  bindSlider("omegaphi1", "omegaphi1-value", (v) =>
    onInitialConditionChange("omegaphi1", v),
  );
  bindSlider("omegaphi2", "omegaphi2-value", (v) =>
    onInitialConditionChange("omegaphi2", v),
  );

  document.getElementById("play-btn").addEventListener("click", onPlay);
  document.getElementById("pause-btn").addEventListener("click", onPause);
  document.getElementById("reset-btn").addEventListener("click", onReset);

  document.getElementById("compare-enable").addEventListener("change", (e) => {
    compare.enabled = e.target.checked;
    if (compare.enabled) {
      compare.enable();
    } else {
      compare.disable();
    }
    updateDataDisplay();
  });

  document.getElementById("compare-delta").addEventListener("input", () => {
    if (compare.enabled) {
      compare.resync();
      updateDataDisplay();
    }
  });

  document.getElementById("compare-field").addEventListener("change", (e) => {
    compare.field = e.target.value;
    if (compare.enabled) {
      compare.resync();
      updateDataDisplay();
    }
  });

  document.getElementById("compare-resync").addEventListener("click", () => {
    if (compare.enabled) {
      compare.resync();
      updateDataDisplay();
    }
  });

  document.getElementById("physics-mode").addEventListener("change", (e) => {
    physicsMode = e.target.value;
    document
      .getElementById("spherical-only")
      .classList.toggle("visible", physicsMode === "spherical");
    rebuildCompareFieldOptions();
    recreatePendulums();
  });
}

// ---------- 数值防护 ----------

function pauseSimulation(message) {
  isPlaying = false;
  const warn = document.getElementById("numerical-warning");
  warn.textContent = message;
  warn.style.display = "block";
}

function hideWarning() {
  document.getElementById("numerical-warning").style.display = "none";
}

// ---------- 场景与主循环 ----------

function onWindowResize() {
  const container = document.getElementById("canvas-container");
  camera.aspect = container.clientWidth / container.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(container.clientWidth, container.clientHeight);
}

function init() {
  const container = document.getElementById("canvas-container");

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a2e);

  camera = new THREE.PerspectiveCamera(
    60,
    container.clientWidth / container.clientHeight,
    0.1,
    100,
  );
  camera.position.set(0, 0, 6);
  camera.lookAt(0, -1, 0);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(container.clientWidth, container.clientHeight);
  renderer.setPixelRatio(window.devicePixelRatio);
  container.appendChild(renderer.domElement);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.target.set(0, -1, 0);

  scene.add(new THREE.AmbientLight(0x404040, 0.6));
  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
  directionalLight.position.set(5, 10, 5);
  scene.add(directionalLight);

  const pivot = new THREE.Mesh(
    new THREE.SphereGeometry(0.08, 16, 16),
    new THREE.MeshStandardMaterial({
      color: 0x607d8b,
      metalness: 0.5,
      roughness: 0.3,
    }),
  );
  scene.add(pivot);

  const gridHelper = new THREE.GridHelper(10, 20, 0x4fc3f7, 0x0f3460);
  gridHelper.position.y = -3;
  scene.add(gridHelper);

  recreatePendulums();

  window.addEventListener("resize", onWindowResize);
  setupControls();
  updateDataDisplay();
  requestAnimationFrame(animate);
}

function animate(currentTime) {
  requestAnimationFrame(animate);

  if (isPlaying) {
    if (lastFrameTime === null) lastFrameTime = currentTime;
    const elapsed = (currentTime - lastFrameTime) / 1000;
    lastFrameTime = currentTime;

    // 子步上限:标签页后台返回时钳制单帧模拟时长,避免长时间冻结
    const substeps = Math.min(Math.ceil(elapsed / DT), MAX_SUBSTEPS);
    for (let i = 0; i < substeps; i++) {
      pendulumA.step();
      if (pendulumB) pendulumB.step();
    }
    simTime += DT * substeps;

    pendulumA.updateVisuals();
    if (pendulumB) pendulumB.updateVisuals();
    pendulumA.pushTrails(simTime);
    if (pendulumB) pendulumB.pushTrails(simTime);

    if (
      !pendulumA.isFiniteState() ||
      (pendulumB && !pendulumB.isFiniteState())
    ) {
      pauseSimulation("数值发散,已自动暂停 — 请点击重置");
    } else {
      updateDataDisplay();
    }
  } else {
    lastFrameTime = null;
  }

  controls.update();
  renderer.render(scene, camera);
}

init();
