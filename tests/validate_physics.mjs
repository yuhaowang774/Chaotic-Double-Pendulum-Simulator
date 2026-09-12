import assert from "node:assert/strict";
import { derivatives, rk4Step, totalEnergy, G } from "../physics.js";

const DT = 0.002;

function makeState(theta, omega) {
  return { theta: [...theta], omega: [...omega] };
}

// 1) 平衡态 N=3(竖直悬挂全 0 与倒立全 π):导数幅值为零
{
  const params = { masses: [1.2, 0.7, 0.5], lengths: [1.1, 0.9, 0.6] };
  for (const theta of [0, Math.PI]) {
    const d = derivatives(makeState([theta, theta, theta], [0, 0, 0]), params);
    for (const [key, arr] of Object.entries(d)) {
      for (let i = 0; i < arr.length; i++) {
        assert.ok(
          Math.abs(arr[i]) < 1e-12,
          `${key}[${i}] 在平衡态应为零,实际 ${arr[i]}`,
        );
      }
    }
  }
  console.log("PASS 平衡态导数为零(N=3)");
}

// 2) 逐点能量相容 N=3:中心差分 dE/dt 残差 < 1e-5(转录错误闸门)
//    h=1e-5 时差分自身噪声 ~4e-7(实测按 h² 缩放),真正的转录错误会给出 O(1e-2)以上残差
{
  const params = { masses: [1.0, 2.3, 0.8], lengths: [1.1, 0.7, 0.9] };
  const h = 1e-5;
  let worst = 0;
  let s = makeState([1.0, -0.7, 0.9], [2.0, -1.5, 0.8]);
  for (let t = 0; t < 5; t++) {
    const d = derivatives(s, params);
    const fwd = makeState(
      s.theta.map((v, i) => v + d.dtheta[i] * h),
      s.omega.map((v, i) => v + d.domega[i] * h),
    );
    const bwd = makeState(
      s.theta.map((v, i) => v - d.dtheta[i] * h),
      s.omega.map((v, i) => v - d.domega[i] * h),
    );
    const e0 = totalEnergy(s, params);
    const residual =
      Math.abs((totalEnergy(fwd, params) - totalEnergy(bwd, params)) / (2 * h * e0));
    worst = Math.max(worst, residual);
    s = makeState(
      s.theta.map((v) => v * -1.1 + 0.3),
      s.omega.map((v) => v * 1.2 - 0.4),
    );
  }
  assert.ok(
    worst < 1e-5,
    `运动方程与能量公式不相容,最差点 |dE/dt|/E = ${worst.toExponential(3)}`,
  );
  console.log(`PASS 运动方程与能量公式逐点相容(N=3,最差 ${worst.toExponential(2)})`);
}

// 3) N=2 回归:混沌初值、dt=0.0005、60 秒,相对漂移 < 1e-6(与旧专用方程同性质)
{
  const params = { masses: [1.0, 2.3], lengths: [1.1, 0.7] };
  const state = makeState(
    [(120 * Math.PI) / 180, (-10 * Math.PI) / 180],
    [0, 0],
  );
  const dtFine = 0.0005;
  const e0 = totalEnergy(state, params);
  for (let i = 0; i < Math.round(60 / dtFine); i++) rk4Step(state, params, dtFine);
  const drift = Math.abs((totalEnergy(state, params) - e0) / e0);
  assert.ok(drift < 1e-6, `N=2 能量漂移过大: ${drift.toExponential(3)}`);
  console.log(`PASS N=2 能量守恒 60s(dt=0.0005),相对漂移 ${drift.toExponential(2)}`);
}

// 4) N=3 守恒:混沌初值、dt=0.0005、60 秒,相对漂移 < 1e-6
{
  const params = { masses: [1.0, 2.3, 0.8], lengths: [1.1, 0.7, 0.9] };
  const state = makeState(
    [(120 * Math.PI) / 180, (-10 * Math.PI) / 180, (45 * Math.PI) / 180],
    [0, 0.5, -1.0],
  );
  const dtFine = 0.0005;
  const e0 = totalEnergy(state, params);
  for (let i = 0; i < Math.round(60 / dtFine); i++) rk4Step(state, params, dtFine);
  const drift = Math.abs((totalEnergy(state, params) - e0) / e0);
  assert.ok(drift < 1e-6, `N=3 能量漂移过大: ${drift.toExponential(3)}`);
  console.log(`PASS N=3 能量守恒 60s(dt=0.0005),相对漂移 ${drift.toExponential(2)}`);
}

// 5) 小角度周期:N=2、m2=1e-3(第二锤近似并入,保证 M 可逆),θ₁ 周期 ≈ 2π√(l₁/g)
{
  const params = { masses: [1.0, 1e-3], lengths: [1.0, 1.0] };
  const state = makeState([0.01, 0], [0, 0]);
  const crossings = [];
  let prev = state.theta[0];
  for (let i = 0; i < Math.round(10 / DT) && crossings.length < 3; i++) {
    rk4Step(state, params, DT);
    if (prev * state.theta[0] < 0) crossings.push(i * DT);
    prev = state.theta[0];
  }
  assert.equal(crossings.length, 3, "10 秒内应检测到 3 次过零");
  const period = crossings[2] - crossings[0];
  const expected = 2 * Math.PI * Math.sqrt(params.lengths[0] / G);
  assert.ok(
    Math.abs(period - expected) / expected < 0.02,
    `周期偏差过大: ${period.toFixed(4)} vs ${expected.toFixed(4)}`,
  );
  console.log(`PASS 小角度周期 ${period.toFixed(4)}s ≈ 理论 ${expected.toFixed(4)}s`);
}

console.log("全部平面物理验证通过");
