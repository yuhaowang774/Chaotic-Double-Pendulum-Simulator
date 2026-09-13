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
const TRAIL_CAPACITY = 80000; // 单条轨迹容量:0.008s 间隔 × 600s = 75000 点,留余量
const TRAIL_PUSH_INTERVAL = 0.008; // 轨迹采点间隔(模拟时间秒),125 点/秒保证光滑
const TRAIL_TIME_MIN = 0.5; // 轨迹留存范围(秒),滑条为对数刻度
const TRAIL_TIME_MAX = 600;
let trailFadeSeconds = 5; // 轨迹留存时长(秒),显示区可调
const MAX_LINKS = 8;
const AUTO_ROTATE_IDLE_MS = 3000; // 鼠标无操作多久后恢复自动环绕
const SUB = ["₁", "₂", "₃", "₄", "₅", "₆", "₇", "₈"];

// 调色板:按杆序在色相环上插值(首锤→末锤)
const PALETTE_A = { rod: 0x4fc3f7, hueStart: 16, hueSpan: 200 }; // 橙 → 蓝
const PALETTE_B = { rod: 0x81c784, hueStart: 110, hueSpan: 200 }; // 绿 → 紫

function linkColor(palette, index, count) {
  const t = count > 1 ? index / (count - 1) : 0;
  return new THREE.Color().setHSL((palette.hueStart + palette.hueSpan * t) / 360, 0.75, 0.5);
}

// 摆参数(A/B 共用,按杆数组);theta/phi 弧度,omega rad/s
const params = {
  masses: [1.0, 1.0],
  lengths: [1.0, 1.0],
  theta: [Math.PI / 2, Math.PI / 2],
  omega: [0, 0],
  phi: [0, 0],
  omegaphi: [0, 0],
};
let linkCount = 2;

let scene, camera, renderer, controls;
let pendulumA = null;
let pendulumB = null; // 混沌对比模式的 B 摆
let physicsMode = "planar"; // "planar" | "spherical"
let uiLevel = "simple"; // "simple" | "advanced"
let simTime = 0;
let isPlaying = false;
let lastFrameTime = null;
let playbackSpeed = 1.0; // 播放速度倍率
let simDebt = 0; // 未满一个步长的模拟时间结转(慢速/变速时保持节拍精确)
let autoOrbitEnabled = true; // 自动环绕用户开关
let lastInteraction = -1e9; // 最近一次鼠标交互时间(rAF 时钟)

function markInteraction() {
  lastInteraction = performance.now();
}

// 数据面板缓存(rebuildParamPanel 时刷新,避免逐帧 getElementById)
let dataThetaSpans = [];
let dataOmegaSpans = [];

// 动态滑条引用(randomize 后同步显示用)
const sliderRefs = {
  lengths: [],
  masses: [],
  theta: [],
  omega: [],
  phi: [],
  omegaphi: [],
};

// ---------- 轨迹:环形缓冲 + 渲染线 ----------

class TrailRing {
  constructor(capacity) {
    this.capacity = capacity;
    this.data = new Float32Array(capacity * 3); // x,y,z 交错存储,便于整段拷贝
    this.t = new Float64Array(capacity);
    this.head = 0;
    this.count = 0;
  }

  push(x, y, z, time) {
    const idx = (this.head + this.count) % this.capacity;
    this.data[idx * 3] = x;
    this.data[idx * 3 + 1] = y;
    this.data[idx * 3 + 2] = z;
    this.t[idx] = time;
    if (this.count < this.capacity) {
      this.count++;
    } else {
      this.head = (this.head + 1) % this.capacity;
    }
  }

  trim(currentTime) {
    while (
      this.count > 0 &&
      currentTime - this.t[this.head] > trailFadeSeconds
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
      opacity: 0.8,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });
    this.line = new THREE.Line(geometry, material);
    this.line.frustumCulled = false;
    scene.add(this.line);
  }

  // 把环形缓冲按“旧→新”顺序摊平进 geometry
  // 环形缓冲拆成两段连续内存,用 set() 原生拷贝(数万点下远快于逐点循环)
  sync() {
    const { ring, positions } = this;
    const n = ring.count;
    if (n === 0) {
      this.line.geometry.setDrawRange(0, 0);
      return;
    }
    const firstLen = Math.min(n, ring.capacity - ring.head);
    positions.set(
      ring.data.subarray(ring.head * 3, (ring.head + firstLen) * 3),
      0,
    );
    if (n > firstLen) {
      positions.set(ring.data.subarray(0, (n - firstLen) * 3), firstLen * 3);
    }
    this.line.geometry.attributes.position.needsUpdate = true;
    this.line.geometry.setDrawRange(0, n);
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

// ---------- 摆体(N 杆) ----------

// ---------- 摆体材质 ----------

const ROD_UP = new THREE.Vector3(0, 1, 0);
const rodDir = new THREE.Vector3();

class Pendulum {
  constructor(scene, palette, mode) {
    this.mode = mode;
    this.palette = palette;
    this.rebuild(scene);
  }

  rebuild(scene) {
    const n = params.lengths.length;
    this.n = n;
    this.state = this.makeInitialState();
    this.rodMaterial = new THREE.MeshStandardMaterial({
      color: 0xb8bcc8,
      metalness: 0.6,
      roughness: 0.35,
    });
    this.rodGeometry = new THREE.CylinderGeometry(0.02, 0.02, 1, 12);
    this.rods = [];
    this.bobs = [];
    this.trails = [];
    this.tip = [];
    for (let i = 0; i < n; i++) {
      const rod = new THREE.Mesh(this.rodGeometry, this.rodMaterial);
      scene.add(rod);
      this.rods.push(rod);

      const bob = new THREE.Mesh(
        new THREE.SphereGeometry(0.1, 24, 24),
        new THREE.MeshStandardMaterial({
          color: linkColor(this.palette, i, n),
          metalness: 0.55,
          roughness: 0.3,
        }),
      );
      scene.add(bob);
      this.bobs.push(bob);

      this.trails.push(new TrailLine(scene, linkColor(this.palette, i, n)));
      this.tip.push([0, 0, 0]);
    }
    this.resetToInitial();
  }

  makeInitialState() {
    if (this.mode === "planar") {
      return { theta: [...params.theta], omega: [...params.omega] };
    }
    return sphericalStateFromAngles(
      {
        theta: [...params.theta],
        phi: [...params.phi],
        omega: [...params.omega],
        omegaphi: [...params.omegaphi],
      },
      params,
    );
  }

  resetToInitial() {
    this.state = this.makeInitialState();
    this.lastTrailPush = -Infinity; // 重置后立即允许采点
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
      return [...this.state.theta, ...this.state.omega].every(Number.isFinite);
    }
    return [...this.state.p.flat(), ...this.state.v.flat()].every(
      Number.isFinite,
    );
  }

  computeTipPositions() {
    let prev = [0, 0, 0];
    for (let i = 0; i < this.n; i++) {
      let pos;
      if (this.mode === "planar") {
        pos = [
          prev[0] + params.lengths[i] * Math.sin(this.state.theta[i]),
          prev[1] - params.lengths[i] * Math.cos(this.state.theta[i]),
          0,
        ];
      } else {
        pos = [...this.state.p[i]];
      }
      this.tip[i] = pos;
      prev = pos;
    }
  }

  updateVisuals() {
    this.computeTipPositions();
    let prev = [0, 0, 0];
    for (let i = 0; i < this.n; i++) {
      const pos = this.tip[i];
      // 圆柱杆:中点定位 + 单位向量定向 + 长度缩放
      const rod = this.rods[i];
      const dx = pos[0] - prev[0];
      const dy = pos[1] - prev[1];
      const dz = pos[2] - prev[2];
      const len = Math.hypot(dx, dy, dz);
      rod.position.set(
        (prev[0] + pos[0]) / 2,
        (prev[1] + pos[1]) / 2,
        (prev[2] + pos[2]) / 2,
      );
      if (len > 1e-9) {
        rodDir.set(dx / len, dy / len, dz / len);
        rod.quaternion.setFromUnitVectors(ROD_UP, rodDir);
      }
      rod.scale.set(1, len, 1);
      this.bobs[i].position.set(pos[0], pos[1], pos[2]);
      prev = pos;
    }
  }

  // 轨迹按固定模拟时间间隔采点(帧率/速度无关),由主循环在子步间调用
  pushTrails(t) {
    if (!document.getElementById("show-trail").checked) return;
    if (t - this.lastTrailPush < TRAIL_PUSH_INTERVAL) return;
    this.lastTrailPush = t;
    this.computeTipPositions();
    for (let i = 0; i < this.n; i++) {
      this.trails[i].ring.push(
        this.tip[i][0], this.tip[i][1], this.tip[i][2], t,
      );
    }
  }

  syncTrails(simTime) {
    for (const t of this.trails) {
      t.ring.trim(simTime);
      t.sync();
    }
  }

  clearTrails() {
    for (const t of this.trails) t.clear();
  }

  dispose(scene) {
    for (const rod of this.rods) scene.remove(rod);
    this.rodGeometry.dispose();
    this.rodMaterial.dispose();
    for (const bob of this.bobs) {
      scene.remove(bob);
      bob.geometry.dispose();
      bob.material.dispose();
    }
    for (const t of this.trails) t.dispose(scene);
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
  let thetas, omegas, energy;
  if (physicsMode === "planar") {
    thetas = pendulumA.state.theta;
    omegas = pendulumA.state.omega;
    energy = totalEnergy(pendulumA.state, params);
  } else {
    const a = sphericalAnglesFromState(pendulumA.state, params);
    thetas = a.theta;
    omegas = a.omega;
    energy = sphericalEnergy(pendulumA.state, params);
  }
  for (let i = 0; i < dataThetaSpans.length; i++) {
    dataThetaSpans[i].textContent = normalizeDeg(thetas[i]).toFixed(1);
    dataOmegaSpans[i].textContent = omegas[i].toFixed(3);
  }
  document.getElementById("total-energy").textContent = energy.toFixed(3);
  document.getElementById("run-time").textContent = simTime.toFixed(2);

  if (pendulumB) {
    let dFirst, dLast;
    if (physicsMode === "planar") {
      dFirst =
        (Math.abs(pendulumA.state.theta[0] - pendulumB.state.theta[0]) * 180) /
        Math.PI;
      dLast =
        (Math.abs(
          pendulumA.state.theta[linkCount - 1] -
            pendulumB.state.theta[linkCount - 1],
        ) *
          180) /
        Math.PI;
    } else {
      const aa = sphericalAnglesFromState(pendulumA.state, params);
      const ab = sphericalAnglesFromState(pendulumB.state, params);
      dFirst = (Math.abs(aa.theta[0] - ab.theta[0]) * 180) / Math.PI;
      dLast =
        (Math.abs(aa.theta[linkCount - 1] - ab.theta[linkCount - 1]) * 180) /
        Math.PI;
    }
    document.getElementById("delta-theta-first").textContent =
      formatDelta(dFirst);
    document.getElementById("delta-theta-last").textContent =
      formatDelta(dLast);
  }
}

// ---------- 混沌对比 ----------

const compare = {
  enabled: false,
  field: "theta1", // "theta1" | "phi2" | "omega1" | "omegaphi3" …

  get delta() {
    const raw = parseFloat(document.getElementById("compare-delta").value);
    return Number.isFinite(raw) ? raw : 0;
  },

  // 角度差(θ/φ)输入为度,ω 差输入为 rad/s
  deltaRadians() {
    return this.field.startsWith("omega")
      ? this.delta
      : (this.delta * Math.PI) / 180;
  },

  parseField() {
    const m = this.field.match(/^(theta|phi|omega|omegaphi)(\d+)$/);
    return m ? { type: m[1], index: Number(m[2]) - 1 } : { type: "theta", index: 0 };
  },

  resync() {
    if (!pendulumB) return;
    const { type, index } = this.parseField();
    if (physicsMode === "planar") {
      pendulumB.state = {
        theta: [...pendulumA.state.theta],
        omega: [...pendulumA.state.omega],
      };
      pendulumB.state[type][index] += this.deltaRadians();
    } else {
      // 球面:反解 A 的角度初值,施加 δ 后重建笛卡尔状态
      const angles = sphericalAnglesFromState(pendulumA.state, params);
      angles[type][index] += this.deltaRadians();
      pendulumB.state = sphericalStateFromAngles(angles, params);
    }
    pendulumB.clearTrails();
    pendulumB.updateVisuals();
  },

  enable() {
    pendulumB = new Pendulum(scene, PALETTE_B, physicsMode);
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

function compareFieldOptions() {
  const options = [];
  for (let i = 0; i < linkCount; i++) {
    options.push([`theta${i + 1}`, `角度 θ${SUB[i]}`]);
    options.push([`omega${i + 1}`, `角速度 ω${SUB[i]}`]);
    if (physicsMode === "spherical") {
      options.push([`phi${i + 1}`, `方位角 φ${SUB[i]}`]);
      options.push([`omegaphi${i + 1}`, `角速度 ωφ${SUB[i]}`]);
    }
  }
  return options;
}

function rebuildCompareFieldOptions() {
  const select = document.getElementById("compare-field");
  const options = compareFieldOptions();
  select.innerHTML = options
    .map(([value, label]) => `<option value="${value}">${label}</option>`)
    .join("");
  if (!options.some(([value]) => value === compare.field)) {
    compare.field = options[0][0];
  }
  select.value = compare.field;
}

// ---------- 动态面板(每杆一组滑条) ----------

function createSlider(container, labelText, min, max, step, value, format, onChange) {
  const group = document.createElement("div");
  group.className = "control-group";
  const label = document.createElement("label");
  const span = document.createElement("span");
  span.textContent = format(value);
  label.append(document.createTextNode(labelText), span);
  const input = document.createElement("input");
  input.type = "range";
  input.min = min;
  input.max = max;
  input.step = step;
  input.value = value;
  input.addEventListener("input", () => {
    const v = parseFloat(input.value);
    span.textContent = format(v);
    onChange(v);
  });
  group.append(label, input);
  container.append(group);
  return { input, span };
}

function rebuildParamPanel() {
  const n = linkCount;
  for (const key of Object.keys(sliderRefs)) sliderRefs[key] = [];

  // 摆参数区:每杆 L/m
  const paramsBox = document.getElementById("links-params");
  paramsBox.innerHTML = "";
  for (let i = 0; i < n; i++) {
    const card = document.createElement("div");
    card.className = "link-card";
    const head = document.createElement("h4");
    head.textContent = `杆 ${i + 1}`;
    const grid = document.createElement("div");
    grid.className = "link-grid";
    card.append(head, grid);
    sliderRefs.lengths[i] = createSlider(
      grid,
      `杆长 L${SUB[i]} (m): `,
      0.5, 2, 0.1, params.lengths[i],
      (v) => v.toFixed(1),
      (v) => onPhysicalParamChange("lengths", i, v),
    );
    sliderRefs.masses[i] = createSlider(
      grid,
      `质量 m${SUB[i]} (kg): `,
      0.1, 5, 0.1, params.masses[i],
      (v) => v.toFixed(1),
      (v) => onPhysicalParamChange("masses", i, v),
    );
    paramsBox.append(card);
  }

  // 初始条件区:每杆 θ/ω(+球面专属 φ/ωφ,CSS 控制显隐)
  const initBox = document.getElementById("links-initial");
  initBox.innerHTML = "";
  for (let i = 0; i < n; i++) {
    const card = document.createElement("div");
    card.className = "link-card";
    const head = document.createElement("h4");
    head.textContent = `杆 ${i + 1}`;
    const grid = document.createElement("div");
    grid.className = "link-grid";
    card.append(head, grid);
    sliderRefs.theta[i] = createSlider(
      grid,
      `角度 θ${SUB[i]} (°): `,
      -180, 180, 1, (params.theta[i] * 180) / Math.PI,
      (v) => `${v}`,
      (v) => onInitialParamChange("theta", i, (v * Math.PI) / 180),
    );
    sliderRefs.omega[i] = createSlider(
      grid,
      `角速度 ω${SUB[i]}: `,
      -10, 10, 0.1, params.omega[i],
      (v) => v.toFixed(1),
      (v) => onInitialParamChange("omega", i, v),
    );
    const sphGrid = document.createElement("div");
    sphGrid.className = "link-grid spherical-slider-group";
    sliderRefs.phi[i] = createSlider(
      sphGrid,
      `方位角 φ${SUB[i]} (°): `,
      -180, 180, 1, (params.phi[i] * 180) / Math.PI,
      (v) => `${v}`,
      (v) => onInitialParamChange("phi", i, (v * Math.PI) / 180),
    );
    sliderRefs.omegaphi[i] = createSlider(
      sphGrid,
      `角速度 ωφ${SUB[i]}: `,
      -10, 10, 0.1, params.omegaphi[i],
      (v) => v.toFixed(1),
      (v) => onInitialParamChange("omegaphi", i, v),
    );
    card.append(sphGrid);
    initBox.append(card);
  }

  // 数据面板:θ/ω 行
  const dataBox = document.getElementById("links-data");
  dataBox.innerHTML = "";
  dataThetaSpans = [];
  dataOmegaSpans = [];
  for (let i = 0; i < n; i++) {
    const pT = document.createElement("p");
    const sT = document.createElement("span");
    sT.textContent = "--";
    pT.append(`角度 θ${SUB[i]}: `, sT, " °");
    const pO = document.createElement("p");
    const sO = document.createElement("span");
    sO.textContent = "--";
    pO.append(`角速度 ω${SUB[i]}: `, sO);
    dataBox.append(pT, pO);
    dataThetaSpans.push(sT);
    dataOmegaSpans.push(sO);
  }
}

function onPhysicalParamChange(kind, index, value) {
  params[kind][index] = value;
  clearAllTrails();
  pendulumA.updateVisuals();
  if (pendulumB) pendulumB.updateVisuals();
  updateDataDisplay();
}

// 初值滑条(θ/φ/ω/ωφ):仅暂停时可调;按滑条整体重建初始状态,
// B 摆(若启用)经 resync 自动跟随保持 δ
function onInitialParamChange(kind, index, value) {
  params[kind][index] = value;
  if (isPlaying) return;
  pendulumA.resetToInitial();
  clearAllTrails();
  pendulumA.updateVisuals();
  if (compare.enabled) compare.resync();
  updateDataDisplay();
}

// 杆数变更:参数数组伸缩(保留原值,新杆默认 L=1、m=1、θ=90°、φ=0、角速度 0)
function setLinkCount(n) {
  linkCount = n;
  for (const [key, fill] of [
    ["masses", 1.0],
    ["lengths", 1.0],
    ["theta", Math.PI / 2],
    ["omega", 0],
    ["phi", 0],
    ["omegaphi", 0],
  ]) {
    const arr = params[key];
    while (arr.length < n) arr.push(fill);
    arr.length = n;
  }
  rebuildParamPanel();
  rebuildCompareFieldOptions();
  recreatePendulums();
}

// 一键随机初始值:θ/ω 全杆随机(球面模式含 φ/ωφ),量化到滑条步长;
// 对比模式开启时 B 摆自动跟随保持 δ
function randomizeInitialConditions() {
  const rand = (min, max) => Math.random() * (max - min) + min;
  for (let i = 0; i < linkCount; i++) {
    params.theta[i] = Math.round(rand(-180, 180)) * (Math.PI / 180);
    params.omega[i] = Math.round(rand(-10, 10) * 10) / 10;
    if (physicsMode === "spherical") {
      params.phi[i] = Math.round(rand(-180, 180)) * (Math.PI / 180);
      params.omegaphi[i] = Math.round(rand(-10, 10) * 10) / 10;
    }
  }
  pendulumA.resetToInitial();
  clearAllTrails();
  pendulumA.updateVisuals();
  if (compare.enabled) compare.resync();
  syncSlidersFromParams();
  updateDataDisplay();
}

function syncSlidersFromParams() {
  const set = (kind, i, value, format) => {
    const ref = sliderRefs[kind][i];
    if (!ref) return;
    ref.input.value = value;
    ref.span.textContent = format(value);
  };
  const deg = (rad) => Math.round((rad * 180) / Math.PI);
  for (let i = 0; i < linkCount; i++) {
    set("theta", i, deg(params.theta[i]), (v) => `${v}`);
    set("omega", i, params.omega[i], (v) => v.toFixed(1));
    if (physicsMode === "spherical") {
      set("phi", i, deg(params.phi[i]), (v) => `${v}`);
      set("omegaphi", i, params.omegaphi[i], (v) => v.toFixed(1));
    }
  }
}

// 物理模式切换(分段按钮)
function setPhysicsMode(mode) {
  if (physicsMode === mode) return;
  physicsMode = mode;
  document
    .querySelectorAll("#mode-segment button")
    .forEach((btn) => btn.classList.toggle("active", btn.dataset.mode === mode));
  document
    .getElementById("control-panel")
    .classList.toggle("mode-spherical", mode === "spherical");
  rebuildCompareFieldOptions();
  recreatePendulums();
}

function recreatePendulums() {
  setPlaying(false);
  simTime = 0;
  if (pendulumA) pendulumA.dispose(scene);
  if (pendulumB) {
    pendulumB.dispose(scene);
    pendulumB = null;
  }
  pendulumA = new Pendulum(scene, PALETTE_A, physicsMode);
  pendulumA.updateVisuals();
  if (compare.enabled) {
    pendulumB = new Pendulum(scene, PALETTE_B, physicsMode);
    compare.resync();
  }
  updateDataDisplay();
}

function clearAllTrails() {
  pendulumA.clearTrails();
  if (pendulumB) pendulumB.clearTrails();
}

// 留存时长变化时立即重修剪并重绘全部轨迹(暂停状态下也生效)
function syncAllTrails() {
  for (const pend of [pendulumA, pendulumB]) {
    if (!pend) continue;
    for (const t of pend.trails) {
      t.ring.trim(simTime);
      t.sync();
    }
  }
}

// ---------- 控制按钮与数值防护 ----------

function setPlaying(playing) {
  isPlaying = playing;
  if (!playing) simDebt = 0; // 暂停时丢弃未结转的模拟时间
  document.getElementById("play-toggle-btn").textContent = playing
    ? "暂停"
    : "播放";
}

function togglePlay() {
  setPlaying(!isPlaying);
  if (isPlaying) hideWarning();
}

function onReset() {
  setPlaying(false);
  simTime = 0;
  pendulumA.resetToInitial();
  clearAllTrails();
  pendulumA.updateVisuals();
  if (compare.enabled) compare.resync();
  updateDataDisplay();
}

function pauseSimulation(message) {
  setPlaying(false);
  const warn = document.getElementById("numerical-warning");
  warn.textContent = message;
  warn.style.display = "block";
}

function hideWarning() {
  document.getElementById("numerical-warning").style.display = "none";
}

// 轨迹留存:滑条 0-100 对数映射到 0.5-600 秒(10 分钟),低段细调、高段直达
const sliderToSeconds = (v) =>
  TRAIL_TIME_MIN * Math.pow(TRAIL_TIME_MAX / TRAIL_TIME_MIN, v / 100);
const secondsToSlider = (t) =>
  (100 * Math.log(t / TRAIL_TIME_MIN)) / Math.log(TRAIL_TIME_MAX / TRAIL_TIME_MIN);
const formatTrailTime = (t) =>
  t < 10 ? `${t.toFixed(1)}s` : `${Math.round(t)}s`;

function setupControls() {
  document.getElementById("play-toggle-btn").addEventListener("click", togglePlay);
  document.getElementById("reset-btn").addEventListener("click", onReset);

  document.getElementById("speed").addEventListener("input", (e) => {
    playbackSpeed = parseFloat(e.target.value);
    document.getElementById("speed-value").textContent =
      `${playbackSpeed.toFixed(1)}x`;
  });

  document.getElementById("trail-time").addEventListener("input", (e) => {
    trailFadeSeconds = sliderToSeconds(parseFloat(e.target.value));
    document.getElementById("trail-time-value").textContent =
      formatTrailTime(trailFadeSeconds);
    syncAllTrails();
  });

  document.getElementById("auto-orbit").addEventListener("change", (e) => {
    autoOrbitEnabled = e.target.checked;
    if (!autoOrbitEnabled) controls.autoRotate = false;
  });

  const trailTimeInput = document.getElementById("trail-time");
  trailTimeInput.value = secondsToSlider(trailFadeSeconds);
  document.getElementById("trail-time-value").textContent =
    formatTrailTime(trailFadeSeconds);

  document.getElementById("link-count").addEventListener("input", (e) => {
    const n = parseInt(e.target.value, 10);
    document.getElementById("link-count-value").textContent = n;
    setLinkCount(n);
  });

  document.getElementById("random-btn").addEventListener("click", () => {
    randomizeInitialConditions();
  });

  document
    .querySelectorAll("#mode-segment button")
    .forEach((btn) =>
      btn.addEventListener("click", () => setPhysicsMode(btn.dataset.mode)),
    );

  document
    .querySelectorAll("#ui-level-segment button")
    .forEach((btn) =>
      btn.addEventListener("click", () => {
        uiLevel = btn.dataset.level;
        document
          .querySelectorAll("#ui-level-segment button")
          .forEach((b) => b.classList.toggle("active", b === btn));
        document
          .getElementById("control-panel")
          .classList.toggle("ui-advanced", uiLevel === "advanced");
      }),
    );

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
  scene.background = new THREE.Color(0x000000);

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

  // 空闲自动环绕:鼠标按下(画布/面板)或滚轮缩放时暂停,
  // 松手 3 秒无操作后恢复;每帧由 animate 统一裁决
  controls.autoRotateSpeed = 0.8;
  renderer.domElement.addEventListener("pointerdown", markInteraction);
  renderer.domElement.addEventListener("wheel", markInteraction, { passive: true });
  document
    .getElementById("control-panel")
    .addEventListener("pointerdown", markInteraction);
  window.addEventListener("pointerup", markInteraction);

  scene.add(new THREE.AmbientLight(0x404040, 0.6));
  const keyLight = new THREE.DirectionalLight(0xffffff, 0.9);
  keyLight.position.set(5, 10, 5);
  scene.add(keyLight);
  const rimLight = new THREE.DirectionalLight(0x8088b0, 0.5);
  rimLight.position.set(-6, 4, -5);
  scene.add(rimLight);

  const pivot = new THREE.Mesh(
    new THREE.SphereGeometry(0.07, 20, 20),
    new THREE.MeshStandardMaterial({
      color: 0xd8dae4,
      metalness: 0.6,
      roughness: 0.3,
    }),
  );
  scene.add(pivot);

  // 网格地面:单色暗灰,提供 3D 深度参照
  const gridHelper = new THREE.GridHelper(10, 20, 0x555555, 0x222222);
  gridHelper.position.y = -3;
  scene.add(gridHelper);

  rebuildParamPanel();
  rebuildCompareFieldOptions();
  recreatePendulums();

  window.addEventListener("resize", onWindowResize);
  setupControls();
  updateDataDisplay();
  requestAnimationFrame(animate);
}

function animate(currentTime) {
  requestAnimationFrame(animate);

  // 自动环绕:开关开启且 3 秒无鼠标操作时生效
  controls.autoRotate =
    autoOrbitEnabled && currentTime - lastInteraction >= AUTO_ROTATE_IDLE_MS;

  if (isPlaying) {
    if (lastFrameTime === null) lastFrameTime = currentTime;
    const elapsed = (currentTime - lastFrameTime) / 1000;
    lastFrameTime = currentTime;

    // 速度倍率 + 步长债务结转:平均速率精确,慢速不抖动;
    // 子步上限防标签页后台返回时长时间冻结(超限时丢弃积压)
    const target = elapsed * playbackSpeed + simDebt;
    let substeps = Math.floor(target / DT);
    if (substeps > MAX_SUBSTEPS) {
      substeps = MAX_SUBSTEPS;
      simDebt = 0;
    } else {
      simDebt = target - substeps * DT;
    }
    // 轨迹按固定模拟时间间隔采点(每 10 个子步一次),高倍速下采样密度不降级
    const pushEvery = Math.max(1, Math.round(TRAIL_PUSH_INTERVAL / DT));
    for (let i = 0; i < substeps; i++) {
      pendulumA.step();
      if (pendulumB) pendulumB.step();
      if ((i + 1) % pushEvery === 0) {
        const t = simTime + DT * (i + 1);
        pendulumA.pushTrails(t);
        if (pendulumB) pendulumB.pushTrails(t);
      }
    }
    simTime += DT * substeps;
    pendulumA.pushTrails(simTime);
    if (pendulumB) pendulumB.pushTrails(simTime);

    pendulumA.updateVisuals();
    if (pendulumB) pendulumB.updateVisuals();
    pendulumA.syncTrails(simTime);
    if (pendulumB) pendulumB.syncTrails(simTime);

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
