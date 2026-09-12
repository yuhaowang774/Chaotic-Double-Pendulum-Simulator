import assert from "node:assert/strict";
import {
  sphericalStateFromAngles,
  sphericalAnglesFromState,
  sphericalStep,
  sphericalEnergy,
  G,
} from "../spherical.js";

const DT = 0.002;

function anglesToState(params, a) {
  return sphericalStateFromAngles(a, params);
}

// 1) 平衡态:全零角 → 两锤竖直悬挂;100 步后约束/速度/能量均不变
{
  const params = { m1: 1.2, m2: 0.7, l1: 1.1, l2: 0.9 };
  const state = anglesToState(params, {
    theta1: 0, phi1: 0, omega1: 0, omegaphi1: 0,
    theta2: 0, phi2: 0, omega2: 0, omegaphi2: 0,
  });
  assert.ok(Math.abs(state.p1[1] + params.l1) < 1e-12, "p1 应悬挂于 (0,-l1,0)");
  assert.ok(
    Math.abs(state.p2[1] + (params.l1 + params.l2)) < 1e-12,
    "p2 应悬挂于 (0,-(l1+l2),0)",
  );
  const e0 = sphericalEnergy(state, params);
  for (let i = 0; i < 100; i++) sphericalStep(state, params, DT);
  const c1 = 0.5 * (state.p1[0] ** 2 + state.p1[1] ** 2 + state.p1[2] ** 2 - params.l1 ** 2);
  const dx = [state.p2[0] - state.p1[0], state.p2[1] - state.p1[1], state.p2[2] - state.p1[2]];
  const c2 = 0.5 * (dx[0] ** 2 + dx[1] ** 2 + dx[2] ** 2 - params.l2 ** 2);
  assert.ok(Math.abs(c1) < 1e-12 && Math.abs(c2) < 1e-12, "平衡态约束应保持");
  const speed =
    Math.hypot(...state.v1) + Math.hypot(...state.v2);
  assert.ok(speed < 1e-12, `平衡态速度应为零,实际 ${speed}`);
  assert.ok(
    Math.abs(sphericalEnergy(state, params) - e0) < 1e-12,
    "平衡态能量应不变",
  );
  console.log("PASS 平衡态(静止悬挂,约束与能量不变)");
}

// 2) 约束残差:混沌初值 60 秒(dt=0.002,生产步长)后 |c1|、|c2| < 1e-9
// 3) 能量守恒:同初值、dt=0.0005 积分 60 秒,相对漂移 < 1e-6
//    (RK4 非辛,生产 dt=0.002 下 60s 漂移 ~1e-5 量级,与平面模块同性质;守恒质量用细步长把关)
{
  const params = { m1: 1.0, m2: 2.3, l1: 1.1, l2: 0.7 };
  const angles = {
    theta1: (120 * Math.PI) / 180,
    phi1: (30 * Math.PI) / 180,
    omega1: 0,
    omegaphi1: 0.5,
    theta2: (-10 * Math.PI) / 180,
    phi2: (-50 * Math.PI) / 180,
    omega2: 1.0,
    omegaphi2: 0,
  };
  const state = anglesToState(params, angles);
  const e0 = sphericalEnergy(state, params);
  const dtFine = 0.0005; // 守恒质量用细步长把关;生产 dt=0.002 下 60s 漂移 ~1.5e-5(RK4 截断)
  const steps = Math.round(60 / dtFine);
  for (let i = 0; i < steps; i++) sphericalStep(state, params, dtFine);
  const c1 = 0.5 * (state.p1[0] ** 2 + state.p1[1] ** 2 + state.p1[2] ** 2 - params.l1 ** 2);
  const dx = [state.p2[0] - state.p1[0], state.p2[1] - state.p1[1], state.p2[2] - state.p1[2]];
  const c2 = 0.5 * (dx[0] ** 2 + dx[1] ** 2 + dx[2] ** 2 - params.l2 ** 2);
  assert.ok(Math.abs(c1) < 1e-9, `c1 残差过大: ${c1.toExponential(3)}`);
  assert.ok(Math.abs(c2) < 1e-9, `c2 残差过大: ${c2.toExponential(3)}`);
  console.log("PASS 约束残差 60s(dt=0.002,|c1|、|c2| < 1e-9)");
  const drift = Math.abs((sphericalEnergy(state, params) - e0) / e0);
  assert.ok(drift < 1e-6, `能量漂移过大: ${drift.toExponential(3)}`);
  console.log(`PASS 能量守恒 60s(dt=0.0005),相对漂移 ${drift.toExponential(2)}`);
}

// 4) 平面极限:φ≡0、L2→0(第二臂并入),小角度 θ1 周期 ≈ 2π√(l1/g),且全程 |z|≈0
{
  const params = { m1: 1.0, m2: 1.0, l1: 1.0, l2: 0.001 };
  const state = anglesToState(params, {
    theta1: 0.01, phi1: 0, omega1: 0, omegaphi1: 0,
    theta2: 0, phi2: 0, omega2: 0, omegaphi2: 0,
  });
  const crossings = [];
  let prevX = state.p1[0]; // 球面极角恒非负,改用摆锤 x 坐标过零(每半周期一次)
  let maxZ = 0;
  const steps = Math.round(10 / DT);
  for (let i = 0; i < steps && crossings.length < 3; i++) {
    sphericalStep(state, params, DT);
    maxZ = Math.max(maxZ, Math.abs(state.p1[2]));
    if (prevX * state.p1[0] < 0) crossings.push(i * DT);
    prevX = state.p1[0];
  }
  assert.equal(crossings.length, 3, "10 秒内应检测到 3 次过零");
  const period = crossings[2] - crossings[0];
  const expected = 2 * Math.PI * Math.sqrt(params.l1 / G);
  assert.ok(
    Math.abs(period - expected) / expected < 0.02,
    `周期偏差过大: ${period.toFixed(4)} vs ${expected.toFixed(4)}`,
  );
  assert.ok(maxZ < 1e-9, `平面极限下不应有 z 分量,实际 max|z|=${maxZ.toExponential(3)}`);
  console.log(`PASS 平面极限:周期 ${period.toFixed(4)}s ≈ 理论 ${expected.toFixed(4)}s,且保持平面`);
}

// 5) 非平面性:方位角速度 ωφ1 ≠ 0 时摆锤必须离开 x-y 平面(抓方位角映射符号错)
{
  const params = { m1: 1.0, m2: 1.0, l1: 1.0, l2: 1.0 };
  const state = anglesToState(params, {
    theta1: 1.0, phi1: 0, omega1: 0, omegaphi1: 2.0,
    theta2: 0, phi2: 0, omega2: 0, omegaphi2: 0,
  });
  let maxZ = 0;
  const steps = Math.round(5 / DT);
  for (let i = 0; i < steps; i++) {
    sphericalStep(state, params, DT);
    maxZ = Math.max(maxZ, Math.abs(state.p1[2]));
  }
  assert.ok(maxZ > 0.05, `方位运动应产生 z 分量,实际 max|z|=${maxZ.toExponential(3)}`);
  console.log(`PASS 非平面性:max|z1| = ${maxZ.toFixed(3)} > 0.05`);
}

console.log("全部球面物理验证通过");
