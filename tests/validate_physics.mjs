import assert from "node:assert/strict";
import { derivatives, rk4Step, totalEnergy, G } from "../physics.js";

// 1) 平衡态(竖直悬挂 θ=0 与倒立 θ=π)导数幅值为零(容差容纳 -0 与浮点噪声)
{
  const params = { m1: 1.2, m2: 0.7, l1: 1.1, l2: 0.9 };
  for (const theta of [0, Math.PI]) {
    const d = derivatives(
      { theta1: theta, theta2: theta, omega1: 0, omega2: 0 },
      params,
    );
    for (const [key, value] of Object.entries(d)) {
      assert.ok(
        Math.abs(value) < 1e-12,
        `${key} 在平衡态应为零,实际 ${value}`,
      );
    }
  }
  console.log("PASS 平衡态导数为零");
}

// 2) 逐点能量相容性:dE/dt = ∇E·(dx/dt) 应恒等于 0。
//    中心差分 [E(s+h·f)−E(s−h·f)]/(2h) 的残差若显著非零,说明运动方程与能量公式
//    转录不一致(与混沌无关,是转录错误的直接闸门)。实测噪声 ~3e-7,容差 1e-5。
{
  const params = { m1: 1.0, m2: 2.3, l1: 1.1, l2: 0.7 };
  const h = 1e-4;
  let worst = 0;
  let s = { theta1: 1.0, theta2: -0.7, omega1: 2.0, omega2: -1.5 };
  for (let i = 0; i < 5; i++) {
    const d = derivatives(s, params);
    const fwd = {
      theta1: s.theta1 + d.dtheta1 * h,
      theta2: s.theta2 + d.dtheta2 * h,
      omega1: s.omega1 + d.domega1 * h,
      omega2: s.omega2 + d.domega2 * h,
    };
    const bwd = {
      theta1: s.theta1 - d.dtheta1 * h,
      theta2: s.theta2 - d.dtheta2 * h,
      omega1: s.omega1 - d.domega1 * h,
      omega2: s.omega2 - d.domega2 * h,
    };
    const e0 = totalEnergy(s, params);
    const residual =
      Math.abs((totalEnergy(fwd, params) - totalEnergy(bwd, params)) / (2 * h * e0));
    worst = Math.max(worst, residual);
    s = {
      theta1: s.theta1 * 1.3 + 0.2,
      theta2: s.theta2 * -0.8 - 0.1,
      omega1: s.omega1 * -1.1 + 0.4,
      omega2: s.omega2 * 1.2 - 0.3,
    };
  }
  assert.ok(
    worst < 1e-5,
    `运动方程与能量公式不相容,最差点 |dE/dt|/E = ${worst.toExponential(3)}`,
  );
  console.log(`PASS 运动方程与能量公式逐点相容(最差 ${worst.toExponential(2)})`);
}

// 3) 非对称参数能量守恒:混沌初值、dt=0.0005 积分 60 秒,相对漂移 < 1e-6。
//    (生产页面用 dt=0.002,60s 实测漂移 ~4e-5,能量面板 3 位小数下不可见)
{
  const params = { m1: 1.0, m2: 2.3, l1: 1.1, l2: 0.7 };
  const state = {
    theta1: (120 * Math.PI) / 180,
    theta2: (-10 * Math.PI) / 180,
    omega1: 0,
    omega2: 0,
  };
  const dt = 0.0005;
  const e0 = totalEnergy(state, params);
  const steps = Math.round(60 / dt);
  for (let i = 0; i < steps; i++) rk4Step(state, params, dt);
  const drift = Math.abs((totalEnergy(state, params) - e0) / e0);
  assert.ok(drift < 1e-6, `能量漂移过大: ${drift.toExponential(3)}`);
  console.log(`PASS 能量守恒 60s(dt=0.0005),相对漂移 ${drift.toExponential(2)}`);
}

// 4) 小角度单摆极限(m2=0):θ₁ 周期 ≈ 2π√(l₁/g),误差 < 2%
{
  const params = { m1: 1.0, m2: 0, l1: 1.0, l2: 1.0 };
  const state = { theta1: 0.01, theta2: 0, omega1: 0, omega2: 0 };
  const crossings = [];
  let prev = state.theta1;
  const steps = Math.round(10 / 0.002);
  for (let i = 0; i < steps && crossings.length < 3; i++) {
    rk4Step(state, params, 0.002);
    if (prev * state.theta1 < 0) crossings.push(i * 0.002);
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

console.log("全部物理验证通过");
