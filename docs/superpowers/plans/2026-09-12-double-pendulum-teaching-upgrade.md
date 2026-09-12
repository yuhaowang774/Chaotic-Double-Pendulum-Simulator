# 双摆模拟器教学升级 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把等质量简化的双摆模拟器升级为通用非对称双摆(独立 m₁/m₂、L₁/L₂),新增"混沌对比"教学模式,并完成角度显示、对象重建、轨迹缓冲、数值防护等基础修复。

**Architecture:** 物理与渲染解耦:新建无 DOM 依赖的 `physics.js`(运动方程/RK4/能量,可被 Node 直接导入验证),`script.js` 重构为 `Pendulum` 类(可实例化 A/B 两摆)+ 环形缓冲轨迹 + 对比管理。HTML/CSS 增量改造,保持零构建、Live Server 直接打开。

**Tech Stack:** 原生 ES Modules、Three.js r160(jsDelivr importmap,不变)、Node 内置 `node:assert`(测试,无测试框架)。

**规格文档:** `docs/superpowers/specs/2026-09-12-double-pendulum-teaching-upgrade-design.md`(本计划的任务与其章节一一对应)。

**环境备注(重要):**
- 本机 shell 曾出现 `D:\Git\bin\bash.exe ENOENT`(Git Bash 启动器丢失)。每个 commit 步骤执行前先跑 `git --version` 探测;若 shell 不可用,**跳过该 commit 并继续**,在任务 6 结束后统一补提交。
- 浏览器验证用本地服务器(如 VS Code Live Server,项目 `.vscode` 已有配置)或 `npx serve`;执行者可用浏览器自动化工具完成检查。

---

## File Structure(改造全景)

| 文件 | 操作 | 职责 |
|---|---|---|
| `physics.js` | 新建 | 纯物理:通用双摆运动方程、RK4、能量。无 DOM。 |
| `tests/validate_physics.mjs` | 新建 | Node 物理验证:平衡态、能量守恒、小角度周期。 |
| `index.html` | 修改 | 摆参数区 4 滑条、p→ω 标签、数值警告条、混沌对比区。 |
| `script.js` | 重写 | `Pendulum` 类(A/B 可实例化)、`TrailRing` 环形轨迹、对比模式、主循环、UI 绑定。 |
| `style.css` | 追加 | 对比模式图例/读数/警告条样式 + 480px 小屏微调。 |

不改动:`medres/`(无引用)、importmap、相机/灯光/网格设置。

---

### Task 0: 基线保护(仅当 shell 可用)

**Files:** 无新文件(git 操作)

- [ ] **Step 1: 初始化仓库并提交现有代码**

```bash
cd "C:\Users\WYH01\Desktop\Project\Chaotic Double Pendulum Simulator"
git --version && git init -b main
git add index.html script.js style.css docs/
git commit -m "chore: 纳入双摆模拟器现状与升级设计文档"
```

Expected: 提交成功(文件数 ≥ 5)。若 shell 不可用(ENOENT),跳过本任务并在文末记录"待补提交"。

---

### Task 1: physics.js — 通用运动方程(TDD)

**Files:**
- Create: `tests/validate_physics.mjs`
- Create: `physics.js`

- [ ] **Step 1: 写失败测试(平衡态导数为零)**

创建 `tests/validate_physics.mjs`:

```js
import assert from "node:assert/strict";
import { derivatives } from "../physics.js";

// 1) 平衡态(竖直悬挂 θ=0 与倒立 θ=π)导数全为零
{
  const params = { m1: 1.2, m2: 0.7, l1: 1.1, l2: 0.9 };
  for (const theta of [0, Math.PI]) {
    const d = derivatives(
      { theta1: theta, theta2: theta, omega1: 0, omega2: 0 },
      params,
    );
    assert.equal(d.dtheta1, 0, "dtheta1");
    assert.equal(d.dtheta2, 0, "dtheta2");
    assert.equal(d.domega1, 0, "domega1");
    assert.equal(d.domega2, 0, "domega2");
  }
  console.log("PASS 平衡态导数为零");
}
```

- [ ] **Step 2: 运行确认失败**

Run: `node tests/validate_physics.mjs`
Expected: FAIL — `Cannot find module '...physics.js'`

- [ ] **Step 3: 实现 physics.js(运动方程部分)**

创建 `physics.js`(注意本任务只含 `G` 与 `derivatives`,RK4/能量在 Task 2 加入):

```js
// 通用非对称双摆物理模块 —— 纯函数,无 DOM 依赖,可被 Node 直接导入验证。
// 约定:θ 从竖直向下方向量起(θ=0 为悬挂平衡态),与页面坐标 x = l·sinθ, y = −l·cosθ 一致。

export const G = 9.8;

/**
 * 运动方程右端项。
 * @param {{theta1:number, theta2:number, omega1:number, omega2:number}} s 状态
 * @param {{m1:number, m2:number, l1:number, l2:number}} p 参数(质量 kg / 杆长 m)
 * @returns {{dtheta1:number, dtheta2:number, domega1:number, domega2:number}}
 *   dtheta* = ω;domega* = 角加速度 α。分母 2m₁+m₂−m₂cos(2Δθ) 对正质量恒为正,无奇点。
 */
export function derivatives(s, p) {
  const { m1, m2, l1, l2 } = p;
  const delta = s.theta1 - s.theta2;
  const sinD = Math.sin(delta);
  const cosD = Math.cos(delta);
  const denom = 2 * m1 + m2 - m2 * Math.cos(2 * delta);

  const domega1 =
    (-G * (2 * m1 + m2) * Math.sin(s.theta1)
      - m2 * G * Math.sin(s.theta1 - 2 * s.theta2)
      - 2 * sinD * m2 * (s.omega2 ** 2 * l2 + s.omega1 ** 2 * l1 * cosD))
    / (l1 * denom);

  const domega2 =
    (2 * sinD
      * (s.omega1 ** 2 * l1 * (m1 + m2)
        + G * (m1 + m2) * Math.cos(s.theta1)
        + s.omega2 ** 2 * l2 * m2 * cosD))
    / (l2 * denom);

  return { dtheta1: s.omega1, dtheta2: s.omega2, domega1, domega2 };
}
```

- [ ] **Step 4: 运行确认通过**

Run: `node tests/validate_physics.mjs`
Expected: 输出 `PASS 平衡态导数为零`,退出码 0

- [ ] **Step 5: Commit**

```bash
git add physics.js tests/validate_physics.mjs
git commit -m "feat: 通用非对称双摆运动方程与平衡态测试"
```

---

### Task 2: RK4 积分器与能量(TDD)

**Files:**
- Modify: `tests/validate_physics.mjs`(追加两个测试)
- Modify: `physics.js`(追加 `rk4Step`、`totalEnergy`)

- [ ] **Step 1: 追加失败测试**

在 `tests/validate_physics.mjs` 顶部修改 import 行并追加两个测试块:

```js
import assert from "node:assert/strict";
import { derivatives, rk4Step, totalEnergy, G } from "../physics.js";

const DT = 0.002;

function integrate(state, params, seconds) {
  const steps = Math.round(seconds / DT);
  for (let i = 0; i < steps; i++) rk4Step(state, params, DT);
}
```

文件末尾追加:

```js
// 2) 非对称参数能量守恒:混沌初值积分 60 秒,相对漂移 < 1e-6
{
  const params = { m1: 1.0, m2: 2.3, l1: 1.1, l2: 0.7 };
  const state = {
    theta1: (120 * Math.PI) / 180,
    theta2: (-10 * Math.PI) / 180,
    omega1: 0,
    omega2: 0,
  };
  const e0 = totalEnergy(state, params);
  integrate(state, params, 60);
  const drift = Math.abs((totalEnergy(state, params) - e0) / e0);
  assert.ok(drift < 1e-6, `能量漂移过大: ${drift.toExponential(3)}`);
  console.log(`PASS 能量守恒 60s,相对漂移 ${drift.toExponential(2)}`);
}

// 3) 小角度单摆极限(m2=0):θ₁ 周期 ≈ 2π√(l₁/g),误差 < 2%
{
  const params = { m1: 1.0, m2: 0, l1: 1.0, l2: 1.0 };
  const state = { theta1: 0.01, theta2: 0, omega1: 0, omega2: 0 };
  const crossings = [];
  let prev = state.theta1;
  const steps = Math.round(10 / DT);
  for (let i = 0; i < steps && crossings.length < 3; i++) {
    rk4Step(state, params, DT);
    if (prev * state.theta1 < 0) crossings.push(i * DT);
    prev = state.theta1;
  }
  assert.equal(crossings.length, 3, "10 秒内应检测到 3 次过零");
  const period = crossings[2] - crossings[0];
  const expected = 2 * Math.PI * Math.sqrt(params.l1 / G);
  assert.ok(
    Math.abs(period - expected) / expected < 0.02,
    `周期偏差过大: ${period.toFixed(4)} vs ${expected.toFixed(4)}`,
  );
  console.log(`PASS 小角度周期 ${period.toFixed(4)}s ≈ 理论 ${expected.toFixed(4)}s`);
}
```

- [ ] **Step 2: 运行确认失败**

Run: `node tests/validate_physics.mjs`
Expected: FAIL — `rk4Step` 未定义(`SyntaxError` 或 undefined 调用错误)

- [ ] **Step 3: 实现 rk4Step 与 totalEnergy**

在 `physics.js` 末尾追加:

```js
/**
 * RK4 单步积分,原地更新 state。
 * @param {{theta1:number, theta2:number, omega1:number, omega2:number}} state
 * @param {{m1:number, m2:number, l1:number, l2:number}} params
 * @param {number} dt 步长(秒)
 */
export function rk4Step(state, params, dt) {
  const k1 = derivatives(state, params);

  const k2 = derivatives({
    theta1: state.theta1 + (k1.dtheta1 * dt) / 2,
    theta2: state.theta2 + (k1.dtheta2 * dt) / 2,
    omega1: state.omega1 + (k1.domega1 * dt) / 2,
    omega2: state.omega2 + (k1.domega2 * dt) / 2,
  }, params);

  const k3 = derivatives({
    theta1: state.theta1 + (k2.dtheta1 * dt) / 2,
    theta2: state.theta2 + (k2.dtheta2 * dt) / 2,
    omega1: state.omega1 + (k2.domega1 * dt) / 2,
    omega2: state.omega2 + (k2.domega2 * dt) / 2,
  }, params);

  const k4 = derivatives({
    theta1: state.theta1 + k3.dtheta1 * dt,
    theta2: state.theta2 + k3.dtheta2 * dt,
    omega1: state.omega1 + k3.domega1 * dt,
    omega2: state.omega2 + k3.domega2 * dt,
  }, params);

  state.theta1 += (dt / 6) * (k1.dtheta1 + 2 * k2.dtheta1 + 2 * k3.dtheta1 + k4.dtheta1);
  state.theta2 += (dt / 6) * (k1.dtheta2 + 2 * k2.dtheta2 + 2 * k3.dtheta2 + k4.dtheta2);
  state.omega1 += (dt / 6) * (k1.domega1 + 2 * k2.domega1 + 2 * k3.domega1 + k4.domega1);
  state.omega2 += (dt / 6) * (k1.domega2 + 2 * k2.domega2 + 2 * k3.domega2 + k4.domega2);
  return state;
}

/**
 * 系统总能量 E = T + V(V 零点取悬挂点高度)。
 * T = ½(m₁+m₂)l₁²ω₁² + m₂l₁l₂ω₁ω₂cosΔθ + ½m₂l₂²ω₂²
 * V = −(m₁+m₂)g l₁cosθ₁ − m₂g l₂cosθ₂
 */
export function totalEnergy(s, p) {
  const { m1, m2, l1, l2 } = p;
  const cosDelta = Math.cos(s.theta1 - s.theta2);
  const kinetic =
    0.5 * (m1 + m2) * l1 * l1 * s.omega1 ** 2
    + m2 * l1 * l2 * s.omega1 * s.omega2 * cosDelta
    + 0.5 * m2 * l2 * l2 * s.omega2 ** 2;
  const potential =
    -(m1 + m2) * G * l1 * Math.cos(s.theta1)
    - m2 * G * l2 * Math.cos(s.theta2);
  return kinetic + potential;
}
```

- [ ] **Step 4: 运行确认全部通过**

Run: `node tests/validate_physics.mjs`
Expected:
```
PASS 平衡态导数为零
PASS 能量守恒 60s,相对漂移 x.xxe-xx
PASS 小角度周期 2.00xxs ≈ 理论 2.0074s
全部物理验证通过
```
若能量漂移 > 1e-6:优先把 `DT` 降到 0.001 重跑;仍不达标说明方程转录有误,逐项对照规格 3.1 的公式。

- [ ] **Step 5: Commit**

```bash
git add physics.js tests/validate_physics.mjs
git commit -m "feat: RK4 积分器与能量公式,守恒/周期验证通过"
```

---

### Task 3: index.html — 参数区改造(独立 L₂/m₂、ω 标签、警告条)

**Files:**
- Modify: `index.html`(三处编辑;行号基于当前文件)

此任务后页面仍用旧 `script.js` 也能运行:所有新元素为增量,旧脚本引用的 id(`l1`、`m1`、`theta1/2`、`omega1/2` 等)全部保留。

- [ ] **Step 1: 替换"摆参数"区(当前第 14–38 行)**

删除原"摆参数"整个 `<div class="param-section">`(含 L 和 m 两组),替换为:

```html
        <div class="param-section">
          <h3>摆参数</h3>
          <div class="control-group">
            <label>杆长 L₁ (m): <span id="l1-value">1.0</span></label>
            <input
              type="range"
              id="l1"
              min="0.5"
              max="2"
              step="0.1"
              value="1.0"
            />
          </div>
          <div class="control-group">
            <label>杆长 L₂ (m): <span id="l2-value">1.0</span></label>
            <input
              type="range"
              id="l2"
              min="0.5"
              max="2"
              step="0.1"
              value="1.0"
            />
          </div>
          <div class="control-group">
            <label>质量 m₁ (kg): <span id="m1-value">1.0</span></label>
            <input
              type="range"
              id="m1"
              min="0.1"
              max="5"
              step="0.1"
              value="1.0"
            />
          </div>
          <div class="control-group">
            <label>质量 m₂ (kg): <span id="m2-value">1.0</span></label>
            <input
              type="range"
              id="m2"
              min="0.1"
              max="5"
              step="0.1"
              value="1.0"
            />
          </div>
        </div>
```

- [ ] **Step 2: 改"初始条件"两个动量滑条标签(当前第 64–74 行)**

id 与量程不变,仅改显示文字(p→ω):

```html
          <div class="control-group">
            <label>角速度 ω₁ (rad/s): <span id="omega1-value">0</span></label>
            <input
              type="range"
              id="omega1"
              min="-10"
              max="10"
              step="0.1"
              value="0"
            />
          </div>

          <div class="control-group">
            <label>角速度 ω₂ (rad/s): <span id="omega2-value">0</span></label>
            <input
              type="range"
              id="omega2"
              min="-10"
              max="10"
              step="0.1"
              value="0"
            />
          </div>
```

- [ ] **Step 3: 面板标题下插入数值警告条(当前第 12 行 `<h2>` 之后)**

```html
      <div id="numerical-warning" style="display: none;"></div>
```

- [ ] **Step 4: 浏览器手动验证**

打开页面(Live Server):旧逻辑下拖动 L/m 滑条应无报错(旧脚本不认识 `l2`/`m2`),播放/暂停/重置正常,控制台无红色错误。

- [ ] **Step 5: Commit**

```bash
git add index.html
git commit -m "feat: 面板参数区改造,独立 L2/m2 滑条与角速度标签"
```

---

### Task 4: script.js 重写 — 接入通用方程 + Pendulum 类 + 环形轨迹 + 防护

**Files:**
- Modify: `script.js`(整文件替换,约 420 行)

要点:状态量改为 (θ₁, θ₂, ω₁, ω₂);轨迹改环形缓冲;删除 `rebuildPendulum`(改参数只清轨迹,几何每帧由 `updateVisuals` 重算);角度显示规范化;NaN 自动暂停;每帧子步上限;轨迹线关闭视锥剔除。本任务先实现 A 摆 + 为 B 摆预留 `pendulumB` 空值守卫(对比逻辑 Task 5 加入)。

- [ ] **Step 1: 用以下完整内容替换 script.js**

```js
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { rk4Step, totalEnergy } from "./physics.js";

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

// 摆参数(A/B 共用);theta 单位弧度,omega 单位 rad/s
const params = {
  m1: 1.0,
  m2: 1.0,
  l1: 1.0,
  l2: 1.0,
  theta1: Math.PI / 2,
  theta2: Math.PI / 2,
  omega1: 0,
  omega2: 0,
};

let scene, camera, renderer, controls;
let pendulumA = null;
let pendulumB = null; // 混沌对比模式的 B 摆,Task 5 中创建
let simTime = 0;
let isPlaying = false;
let lastFrameTime = null;

// ---------- 轨迹:环形缓冲 + 渲染线 ----------

class TrailRing {
  constructor(capacity) {
    this.capacity = capacity;
    this.x = new Float32Array(capacity);
    this.y = new Float32Array(capacity);
    this.t = new Float64Array(capacity);
    this.head = 0;
    this.count = 0;
  }

  push(x, y, time) {
    const idx = (this.head + this.count) % this.capacity;
    if (this.count < this.capacity) {
      this.count++;
    } else {
      this.head = (this.head + 1) % this.capacity;
    }
    this.x[idx] = x;
    this.y[idx] = y;
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
      positions[i * 3 + 2] = 0;
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

function setLine(line, x0, y0, x1, y1) {
  const a = line.geometry.attributes.position.array;
  a[0] = x0;
  a[1] = y0;
  a[2] = 0;
  a[3] = x1;
  a[4] = y1;
  a[5] = 0;
  line.geometry.attributes.position.needsUpdate = true;
}

class Pendulum {
  constructor(scene, palette) {
    this.palette = palette;
    this.state = { theta1: 0, theta2: 0, omega1: 0, omega2: 0 };
    // 供 pushTrails 使用的末端坐标缓存
    this.x1 = 0;
    this.y1 = 0;
    this.x2 = 0;
    this.y2 = 0;

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
    this.state.theta1 = params.theta1;
    this.state.theta2 = params.theta2;
    this.state.omega1 = params.omega1;
    this.state.omega2 = params.omega2;
  }

  step() {
    rk4Step(this.state, params, DT);
  }

  isFiniteState() {
    return (
      Number.isFinite(this.state.theta1) &&
      Number.isFinite(this.state.theta2) &&
      Number.isFinite(this.state.omega1) &&
      Number.isFinite(this.state.omega2)
    );
  }

  updateVisuals() {
    const x1 = params.l1 * Math.sin(this.state.theta1);
    const y1 = -params.l1 * Math.cos(this.state.theta1);
    const x2 = x1 + params.l2 * Math.sin(this.state.theta2);
    const y2 = y1 - params.l2 * Math.cos(this.state.theta2);
    setLine(this.rod1, 0, 0, x1, y1);
    setLine(this.rod2, x1, y1, x2, y2);
    this.bob1.position.set(x1, y1, 0);
    this.bob2.position.set(x2, y2, 0);
    this.x1 = x1;
    this.y1 = y1;
    this.x2 = x2;
    this.y2 = y2;
  }

  pushTrails(simTime) {
    if (!document.getElementById("show-trail").checked) return;
    this.trail1.ring.push(this.x1, this.y1, simTime);
    this.trail2.ring.push(this.x2, this.y2, simTime);
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
  deg = ((deg + 180) % 360 + 360) % 360 - 180;
  if (deg === -180) deg = 180;
  return deg;
}

function updateDataDisplay() {
  document.getElementById("current-theta1").textContent =
    normalizeDeg(pendulumA.state.theta1).toFixed(1);
  document.getElementById("current-theta2").textContent =
    normalizeDeg(pendulumA.state.theta2).toFixed(1);
  document.getElementById("current-omega1").textContent =
    pendulumA.state.omega1.toFixed(3);
  document.getElementById("current-omega2").textContent =
    pendulumA.state.omega2.toFixed(3);
  document.getElementById("total-energy").textContent =
    totalEnergy(pendulumA.state, params).toFixed(3);
  document.getElementById("run-time").textContent = simTime.toFixed(2);
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

// 初值滑条(θ/ω):仅暂停时可调;B 摆(若存在)在 Task 5 中接入跟随
function onInitialConditionChange(field, value) {
  params[field] = value;
  if (isPlaying) return;
  pendulumA.state[field] = value;
  clearAllTrails();
  pendulumA.updateVisuals();
  if (pendulumB) pendulumB.updateVisuals();
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
  if (pendulumB) pendulumB.updateVisuals();
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

  document.getElementById("play-btn").addEventListener("click", onPlay);
  document.getElementById("pause-btn").addEventListener("click", onPause);
  document.getElementById("reset-btn").addEventListener("click", onReset);
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

  pendulumA = new Pendulum(scene, PALETTE_A);
  pendulumA.resetToInitial();
  pendulumA.updateVisuals();

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

    if (!pendulumA.isFiniteState() || (pendulumB && !pendulumB.isFiniteState())) {
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
```

- [ ] **Step 2: 浏览器手动验证**

打开页面,逐项确认:
1. 播放后摆动流畅,轨迹渐隐正常(约 5 秒尾迹);
2. 能量读数稳定在小数点后 2–3 位不持续漂移;
3. 把 θ₁ 拖到 −90° 以下再播放,实时数据显示在 (−180°, 180°] 内、无负零怪象;
4. 暂停时拖 θ₁/θ₂/ω 滑条摆体即时重摆;播放中拖 L/m 滑条立即生效且控制台无报错;
5. 质量滑条 m₁=5、m₂=0.1 与 m₁=0.1、m₂=5 运动形态明显不同(通用方程生效);
6. 播放 1–2 分钟无卡顿、无 NaN。

- [ ] **Step 3: Commit**

```bash
git add script.js
git commit -m "feat: 重写渲染层接入通用方程,Pendulum 类/环形轨迹/数值防护"
```

---

### Task 5: 混沌对比模式

**Files:**
- Modify: `index.html`(data-display 前插入对比区)
- Modify: `script.js`(B 摆调色板、compare 对象、四处函数替换、绑定)
- Modify: `style.css`(文件末尾追加)

- [ ] **Step 1: index.html — 在 `<div class="data-display">`(当前"实时数据"区)之前插入**

```html
        <div class="param-section">
          <h3>混沌对比</h3>
          <div class="compare-legend">
            <span class="compare-legend-item">
              <span class="compare-dot compare-dot-a"></span>A 摆
            </span>
            <span class="compare-legend-item">
              <span class="compare-dot compare-dot-b"></span>B 摆
            </span>
          </div>
          <div class="control-group">
            <label>
              <input type="checkbox" id="compare-enable" />
              启用对比摆 B
            </label>
          </div>
          <div class="control-group">
            <label>初始差值 δ: <input type="number" id="compare-delta" value="0.1" min="0" step="0.1" /></label>
          </div>
          <div class="control-group">
            <label>施加变量:
              <select id="compare-field">
                <option value="theta1" selected>角度 θ₁</option>
                <option value="theta2">角度 θ₂</option>
                <option value="omega1">角速度 ω₁</option>
                <option value="omega2">角速度 ω₂</option>
              </select>
            </label>
          </div>
          <div class="control-group">
            <button id="compare-resync">重新同步 B</button>
          </div>
          <div id="compare-readout" style="display: none;">
            <p>|Δθ₁|: <span id="delta-theta1">--</span> °</p>
            <p>|Δθ₂|: <span id="delta-theta2">--</span> °</p>
          </div>
          <p class="compare-hint">A/B 仅一个初值相差 δ,观察轨迹如何分裂。</p>
        </div>
```

- [ ] **Step 2: script.js — PALETTE_A 定义之后追加 B 调色板与 compare 对象**

```js
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

  // 角度差输入为度,ω 差输入为 rad/s
  deltaRadians() {
    const raw = this.delta;
    return this.field.startsWith("theta") ? (raw * Math.PI) / 180 : raw;
  },

  resync() {
    if (!pendulumB) return;
    pendulumB.state = { ...pendulumA.state };
    pendulumB.state[this.field] += this.deltaRadians();
    pendulumB.clearTrails();
    pendulumB.updateVisuals();
  },

  enable() {
    pendulumB = new Pendulum(scene, PALETTE_B);
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
```

- [ ] **Step 3: script.js — 用下面版本整体替换 `updateDataDisplay` 与 `onInitialConditionChange`、`onReset`,并在 `setupControls` 末尾追加绑定**

替换 `updateDataDisplay`(增加 B 读数):

```js
function updateDataDisplay() {
  document.getElementById("current-theta1").textContent =
    normalizeDeg(pendulumA.state.theta1).toFixed(1);
  document.getElementById("current-theta2").textContent =
    normalizeDeg(pendulumA.state.theta2).toFixed(1);
  document.getElementById("current-omega1").textContent =
    pendulumA.state.omega1.toFixed(3);
  document.getElementById("current-omega2").textContent =
    pendulumA.state.omega2.toFixed(3);
  document.getElementById("total-energy").textContent =
    totalEnergy(pendulumA.state, params).toFixed(3);
  document.getElementById("run-time").textContent = simTime.toFixed(2);

  if (pendulumB) {
    const d1 =
      Math.abs(pendulumA.state.theta1 - pendulumB.state.theta1) * 180 / Math.PI;
    const d2 =
      Math.abs(pendulumA.state.theta2 - pendulumB.state.theta2) * 180 / Math.PI;
    document.getElementById("delta-theta1").textContent = formatDelta(d1);
    document.getElementById("delta-theta2").textContent = formatDelta(d2);
  }
}
```

替换 `onInitialConditionChange`(增加 B 跟随):

```js
function onInitialConditionChange(field, value) {
  params[field] = value;
  if (isPlaying) return;
  pendulumA.state[field] = value;
  clearAllTrails();
  pendulumA.updateVisuals();
  if (compare.enabled) compare.resync();
  updateDataDisplay();
}
```

替换 `onReset`(B 回到"初值 + δ"):

```js
function onReset() {
  isPlaying = false;
  simTime = 0;
  pendulumA.resetToInitial();
  clearAllTrails();
  pendulumA.updateVisuals();
  if (compare.enabled) compare.resync();
  updateDataDisplay();
}
```

`setupControls()` 函数末尾(`reset-btn` 绑定行之后)追加:

```js
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
```

- [ ] **Step 4: style.css — 文件末尾追加**

```css
/* ===== 混沌对比模式(Task 5 新增)===== */

#compare-resync {
  width: 100%;
  padding: 8px;
  border: none;
  border-radius: 5px;
  background: #7e57c2;
  color: #fff;
  font-weight: bold;
  cursor: pointer;
}

#compare-resync:hover {
  background: #9575cd;
}

#compare-delta {
  width: 80px;
  background: #1a4a7a;
  color: #fff;
  border: 1px solid #4fc3f7;
  border-radius: 4px;
  padding: 3px 6px;
}

#compare-field {
  background: #1a4a7a;
  color: #fff;
  border: 1px solid #4fc3f7;
  border-radius: 4px;
  padding: 3px 6px;
}

#compare-readout p {
  margin-bottom: 4px;
  font-size: 0.8em;
  color: #b0bec5;
}

#compare-readout p span {
  color: #fff;
  font-weight: bold;
}

.compare-legend {
  display: flex;
  gap: 14px;
  margin-bottom: 8px;
}

.compare-legend-item {
  display: flex;
  align-items: center;
  gap: 5px;
  font-size: 0.8em;
  color: #b0bec5;
}

.compare-dot {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  display: inline-block;
}

.compare-dot-a {
  background: #ff5722;
}

.compare-dot-b {
  background: #4caf50;
}

.compare-hint {
  font-size: 0.75em;
  opacity: 0.7;
  margin-top: 6px;
}

#numerical-warning {
  color: #ff5252;
  font-weight: 600;
  font-size: 0.85em;
  margin-bottom: 10px;
}

@media (max-width: 480px) {
  #compare-delta {
    width: 64px;
  }

  #compare-field {
    max-width: 140px;
  }
}
```

- [ ] **Step 5: 浏览器手动验证(对比模式全流程)**

1. 勾选"启用对比摆 B":场景出现绿/紫 B 摆,与 A 几乎重合(θ₁ 差 0.1°),读数 |Δθ₁| ≈ 0.1000;
2. 播放 20–40 秒:两条末端轨迹由重合到明显分裂,|Δθ₁| 持续增大(数量级增长);
3. 点"重新同步 B":B 回到 A 当前状态 + δ,轨迹清空;
4. 暂停拖 θ₁ 滑条:B 跟随保持 δ;修改 δ 或施加变量:B 立即重同步;
5. 取消勾选:B 消失、读数隐藏,无控制台报错;
6. 重置:A 回初值、B 回"初值 + δ"。

- [ ] **Step 6: Commit**

```bash
git add index.html script.js style.css
git commit -m "feat: 混沌对比模式(对比摆 B/差值控制/分歧读数)"
```

---

### Task 6: 最终验收与补提交

**Files:** 无新代码;验收 + git 收尾

- [ ] **Step 1: 跑物理验证脚本**

Run: `node tests/validate_physics.mjs`
Expected: 三个 PASS + `全部物理验证通过`,退出码 0

- [ ] **Step 2: 对照规格 §7 验收清单逐项确认**

| # | 验收项 | 方法 |
|---|---|---|
| 1 | m₁≠m₂、L₁≠L₂ 独立可调,小角度周期符合理论值 | 验证脚本 3 + 浏览器拖滑条观察 |
| 2 | 能量相对漂移 60s < 1e-6,面板能量读数稳定 | 验证脚本 2 + 浏览器播 1 分钟看读数 |
| 3 | δ=0.1° 时 10–30 秒轨迹可见分裂,读数增长 | 对比模式播放观察 |
| 4 | 角度显示无负零/越界;调参不重建对象 | 拖 θ 滑条跨 ±180°;控制台无 dispose 报错 |
| 5 | 触屏可完成全部操作 | 浏览器设备模拟(DevTools 触摸仿真)过一遍:旋转/调参/播放/对比 |
| 6 | 零构建:验证脚本直过、页面直开 | 已覆盖 |

- [ ] **Step 3: 补齐提交(含此前因 shell 不可用跳过的部分)**

```bash
git --version   # 若仍不可用,向用户报告并保留工作区现状
git add -A
git commit -m "chore: 教学升级收尾(验证脚本/样式/文档)"
```

- [ ] **Step 4: 向用户交付**

汇报:改动文件清单、验证脚本输出、验收结果、遗留事项(如 shell/git 问题)。

---

## Self-Review 记录

- **规格覆盖**:规格 §2 文件结构→Task 0–5;§3.1 方程/ω 状态/子步上限→Task 1/2/4;§3.2 能量→Task 2;§3.3 防护→Task 4;§4 对比模式全部控件与行为规则→Task 5(含 δ 单位、自动重同步、初值滑条仅暂停可调);§5 修复清单 6 项→Task 3(1)/4(2,3,4,6)/5(6)/Task 4 验证 5(触屏);§6 验证→Task 1/2/6;§7 验收→Task 6。无缺口。
- **占位符扫描**:无 TBD/TODO;所有代码步骤均为完整代码;"手动验证"步骤给出逐项检查内容。
- **命名一致性**:`derivatives/rk4Step/totalEnergy/G` 导出名、`Pendulum` 方法签名(`resetToInitial/step/isFiniteState/updateVisuals/pushTrails/clearTrails/dispose`)、`compare.{enabled,field,delta,deltaRadians,resync,enable,disable}`、HTML id 与 Task 4/5 引用逐一核对一致。B 摆状态同步不设独立方法,由 Task 5 的 `compare.resync()` 内联完成,Task 4 的 `Pendulum` 无需预留接口。
