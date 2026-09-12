import assert from "node:assert/strict";
import {
  sphericalStateFromAngles,
  sphericalAnglesFromState,
  sphericalStep,
  sphericalEnergy,
  G,
} from "../spherical.js";

const DT = 0.002;

const EQ_ANGLES = {
  theta: [0, 0, 0],
  phi: [0, 0, 0],
  omega: [0, 0, 0],
  omegaphi: [0, 0, 0],
};

// 1) 平衡态 N=3:两……N 锤竖直悬挂;100 步后约束/速度/能量均不变
{
  const params = { masses: [1.2, 0.7, 0.5], lengths: [1.1, 0.9, 0.6] };
  const state = sphericalStateFromAngles(EQ_ANGLES, params);
  for (let i = 0; i < 3; i++) {
    assert.ok(
      Math.abs(state.p[i][1] + params.lengths.slice(0, i + 1).reduce((a, b) => a + b, 0)) < 1e-12,
      `p${i + 1} 应悬挂于累加杆长处`,
    );
  }
  const e0 = sphericalEnergy(state, params);
  for (let i = 0; i < 100; i++) sphericalStep(state, params, DT);
  for (let k = 0; k < 3; k++) {
    const d = [
      state.p[k][0] - (k === 0 ? 0 : state.p[k - 1][0]),
      state.p[k][1] - (k === 0 ? 0 : state.p[k - 1][1]),
      state.p[k][2] - (k === 0 ? 0 : state.p[k - 1][2]),
    ];
    const c = 0.5 * (d[0] ** 2 + d[1] ** 2 + d[2] ** 2 - params.lengths[k] ** 2);
    assert.ok(Math.abs(c) < 1e-12, `平衡态约束 ${k + 1} 应保持,实际 ${c.toExponential(2)}`);
  }
  const speed = state.v.reduce((acc, v) => acc + Math.hypot(...v), 0);
  assert.ok(speed < 1e-12, `平衡态速度应为零,实际 ${speed}`);
  assert.ok(
    Math.abs(sphericalEnergy(state, params) - e0) < 1e-12,
    "平衡态能量应不变",
  );
  console.log("PASS 平衡态(静止悬挂,约束与能量不变,N=3)");
}

// 2) 约束残差 N=3:混沌初值 60 秒(dt=0.002,生产步长)后 |c_k| < 1e-9
{
  const params = { masses: [1.0, 2.3, 0.8], lengths: [1.1, 0.7, 0.9] };
  const state = sphericalStateFromAngles(
    {
      theta: [(120 * Math.PI) / 180, (-10 * Math.PI) / 180, (45 * Math.PI) / 180],
      phi: [(30 * Math.PI) / 180, (-50 * Math.PI) / 180, 0],
      omega: [0, 0.5, -1.0],
      omegaphi: [0.5, 0, 0.3],
    },
    params,
  );
  for (let i = 0; i < Math.round(60 / DT); i++) sphericalStep(state, params, DT);
  let prev = [0, 0, 0];
  let worst = 0;
  for (let k = 0; k < 3; k++) {
    const d = [
      state.p[k][0] - prev[0],
      state.p[k][1] - prev[1],
      state.p[k][2] - prev[2],
    ];
    const c = 0.5 * (d[0] ** 2 + d[1] ** 2 + d[2] ** 2 - params.lengths[k] ** 2);
    worst = Math.max(worst, Math.abs(c));
    prev = state.p[k];
  }
  assert.ok(worst < 1e-9, `N=3 约束残差过大: ${worst.toExponential(3)}`);
  console.log("PASS 约束残差 60s(dt=0.002,N=3,< 1e-9)");
}

// 3) 能量守恒 N=3:同初值、dt=0.0005、60 秒,相对漂移 < 1e-6
{
  const params = { masses: [1.0, 2.3, 0.8], lengths: [1.1, 0.7, 0.9] };
  const state = sphericalStateFromAngles(
    {
      theta: [(120 * Math.PI) / 180, (-10 * Math.PI) / 180, (45 * Math.PI) / 180],
      phi: [(30 * Math.PI) / 180, (-50 * Math.PI) / 180, 0],
      omega: [0, 0.5, -1.0],
      omegaphi: [0.5, 0, 0.3],
    },
    params,
  );
  const dtFine = 0.0005;
  const e0 = sphericalEnergy(state, params);
  for (let i = 0; i < Math.round(60 / dtFine); i++) sphericalStep(state, params, dtFine);
  const drift = Math.abs((sphericalEnergy(state, params) - e0) / e0);
  assert.ok(drift < 1e-6, `N=3 能量漂移过大: ${drift.toExponential(3)}`);
  console.log(`PASS 能量守恒 60s(dt=0.0005,N=3),相对漂移 ${drift.toExponential(2)}`);
}

// 4) N=2 回归:混沌初值 60 秒,约束 < 1e-9 且能量漂移 < 1e-6(dt=0.0005)
{
  const params = { masses: [1.0, 2.3], lengths: [1.1, 0.7] };
  const angles = {
    theta: [(120 * Math.PI) / 180, (-10 * Math.PI) / 180],
    phi: [(30 * Math.PI) / 180, (-50 * Math.PI) / 180],
    omega: [0, 0.5],
    omegaphi: [0, 0],
  };
  const state = sphericalStateFromAngles(angles, params);
  const e0 = sphericalEnergy(state, params);
  const dtFine = 0.0005;
  for (let i = 0; i < Math.round(60 / dtFine); i++) sphericalStep(state, params, dtFine);
  const drift = Math.abs((sphericalEnergy(state, params) - e0) / e0);
  assert.ok(drift < 1e-6, `N=2 能量漂移过大: ${drift.toExponential(3)}`);
  // 约束
  const d = [state.p[1][0] - state.p[0][0], state.p[1][1] - state.p[0][1], state.p[1][2] - state.p[0][2]];
  const c2 = 0.5 * (d[0] ** 2 + d[1] ** 2 + d[2] ** 2 - params.lengths[1] ** 2);
  const c1 = 0.5 * (state.p[0][0] ** 2 + state.p[0][1] ** 2 + state.p[0][2] ** 2 - params.lengths[0] ** 2);
  assert.ok(Math.abs(c1) < 1e-9 && Math.abs(c2) < 1e-9, `N=2 约束残差过大: ${c1.toExponential(2)}, ${c2.toExponential(2)}`);
  console.log(`PASS N=2 回归:能量漂移 ${drift.toExponential(2)},约束 < 1e-9`);
}

// 5) 平面极限:N=2、L2→0、m2 小,小角度 θ1 周期 ≈ 2π√(l1/g),且全程 |z|≈0
{
  const params = { masses: [1.0, 1e-3], lengths: [1.0, 0.001] };
  const state = sphericalStateFromAngles(
    { theta: [0.01, 0], phi: [0, 0], omega: [0, 0], omegaphi: [0, 0] },
    params,
  );
  const crossings = [];
  let prevX = state.p[0][0]; // 球面极角恒非负,用摆锤 x 坐标过零(每半周期一次)
  let maxZ = 0;
  for (let i = 0; i < Math.round(10 / DT) && crossings.length < 3; i++) {
    sphericalStep(state, params, DT);
    maxZ = Math.max(maxZ, Math.abs(state.p[0][2]));
    if (prevX * state.p[0][0] < 0) crossings.push(i * DT);
    prevX = state.p[0][0];
  }
  assert.equal(crossings.length, 3, "10 秒内应检测到 3 次过零");
  const period = crossings[2] - crossings[0];
  const expected = 2 * Math.PI * Math.sqrt(params.lengths[0] / G);
  assert.ok(
    Math.abs(period - expected) / expected < 0.02,
    `周期偏差过大: ${period.toFixed(4)} vs ${expected.toFixed(4)}`,
  );
  assert.ok(maxZ < 1e-9, `平面极限下不应有 z 分量,实际 max|z|=${maxZ.toExponential(3)}`);
  console.log(`PASS 平面极限:周期 ${period.toFixed(4)}s ≈ 理论 ${expected.toFixed(4)}s,且保持平面`);
}

// 6) 非平面性 N=2:ωφ1 ≠ 0 时摆锤必须离开 x-y 平面(抓方位角映射符号错)
{
  const params = { masses: [1.0, 1.0], lengths: [1.0, 1.0] };
  const state = sphericalStateFromAngles(
    { theta: [1.0, 0], phi: [0, 0], omega: [0, 0], omegaphi: [2.0, 0] },
    params,
  );
  let maxZ = 0;
  for (let i = 0; i < Math.round(5 / DT); i++) {
    sphericalStep(state, params, DT);
    maxZ = Math.max(maxZ, Math.abs(state.p[0][2]));
  }
  assert.ok(maxZ > 0.05, `方位运动应产生 z 分量,实际 max|z|=${maxZ.toExponential(3)}`);
  console.log(`PASS 非平面性:max|z1| = ${maxZ.toFixed(3)} > 0.05`);
}

console.log("全部球面物理验证通过");
