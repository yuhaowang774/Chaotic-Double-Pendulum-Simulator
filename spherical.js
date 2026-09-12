// 球面 N 杆链物理模块(N=1–8)—— 笛卡尔坐标 + 指标约减(约束加速度)+ RK4。
// 纯函数,无 DOM 依赖,可被 Node 直接导入验证。
// 坐标系与渲染一致:悬挂点在原点,y 轴向上,重力沿 −y。
// 角度约定:θ 为极角(自竖直向下方向,θ=0 悬挂),φ 为方位角(绕 y 轴)。
// dir(θ,φ) = (sinθcosφ, −cosθ, sinθsinφ):全 φ=0、ωφ=0 时与平面 N 杆链逐点重合。
//
// 积分方案:状态为 N 锤笛卡尔位置/速度;加速度由约束 saddle 点方程解析求解
//   M a = F − Jᵀλ,  J a = −γ,  γ_k = |v_k − v_{k−1}|²(v₀ ≡ 0),
// (J M⁻¹ Jᵀ)λ = J M⁻¹ F + γ 的系数矩阵为三对角(相邻约束共享一锤),dense 高斯消元求解。
// RK4 积分,每步后做微幅稳定化(位置牛顿投影 + 去径向速度,修正量 ~1e-12)。

import { G, solveLinear } from "./physics.js";

export { G };

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function direction(theta, phi) {
  const st = Math.sin(theta);
  const ct = Math.cos(theta);
  return [st * Math.cos(phi), -ct, st * Math.sin(phi)];
}

function rodVectors(s) {
  // d[k] = p_k − p_{k−1}(p₋₁ ≡ 原点),dv[k] = v_k − v_{k−1}(v₋₁ ≡ 0)
  const n = s.p.length;
  const ds = [];
  const dvs = [];
  let prevP = [0, 0, 0];
  let prevV = [0, 0, 0];
  for (let k = 0; k < n; k++) {
    ds.push([s.p[k][0] - prevP[0], s.p[k][1] - prevP[1], s.p[k][2] - prevP[2]]);
    dvs.push([s.v[k][0] - prevV[0], s.v[k][1] - prevV[1], s.v[k][2] - prevV[2]]);
    prevP = s.p[k];
    prevV = s.v[k];
  }
  return { ds, dvs };
}

// 约束雅可比相关量:A = J M⁻¹ Jᵀ(三对角)与 J v
function constraintMatrices(ds, dvs, masses) {
  const n = ds.length;
  const A = [];
  const jv = [];
  for (let k = 0; k < n; k++) {
    const row = new Array(n).fill(0);
    row[k] = dot(ds[k], ds[k]) / masses[k];
    if (k > 0) {
      row[k] += dot(ds[k], ds[k]) / masses[k - 1];
      row[k - 1] = -dot(ds[k], ds[k - 1]) / masses[k - 1];
    }
    if (k + 1 < n) row[k + 1] = -dot(ds[k], ds[k + 1]) / masses[k];
    A.push(row);
    jv.push(dot(ds[k], dvs[k]));
  }
  return { A, jv };
}

/**
 * 约束加速度:a_k = (0,−G,0) − (λ_k d_k − λ_{k+1} d_{k+1})/m_k。
 * rhs_k = J M⁻¹ F + γ = (k=0 时 −G·d₀_y,否则 0) + |dv_k|²。
 */
function accelerations(s, params) {
  const n = params.lengths.length;
  const { masses } = params;
  const { ds, dvs } = rodVectors(s);
  const { A } = constraintMatrices(ds, dvs, masses);
  const b = ds.map((d, k) => (k === 0 ? -G * d[1] : 0) + dot(dvs[k], dvs[k]));
  let lam;
  try {
    lam = solveLinear(A, b);
  } catch {
    return s.p.map(() => [0, -G, 0]); // 退化保护
  }
  return s.p.map((_, k) => {
    const corr = [ds[k][0] * lam[k], ds[k][1] * lam[k], ds[k][2] * lam[k]];
    if (k + 1 < n) {
      corr[0] -= ds[k + 1][0] * lam[k + 1];
      corr[1] -= ds[k + 1][1] * lam[k + 1];
      corr[2] -= ds[k + 1][2] * lam[k + 1];
    }
    return [0 - corr[0] / masses[k], -G - corr[1] / masses[k], 0 - corr[2] / masses[k]];
  });
}

function deriv(s, params) {
  const acc = accelerations(s, params);
  return {
    dp: s.v.map((v) => [...v]),
    dv: acc,
  };
}

function offset(s, k, h) {
  return {
    p: s.p.map((p, i) => [p[0] + h * k.dp[i][0], p[1] + h * k.dp[i][1], p[2] + h * k.dp[i][2]]),
    v: s.v.map((v, i) => [v[0] + h * k.dv[i][0], v[1] + h * k.dv[i][1], v[2] + h * k.dv[i][2]]),
  };
}

// 微幅稳定化:位置牛顿投影(不修正速度)+ 去径向速度。每步违反量 ~1e-12。
function stabilize(s, params) {
  const { masses, lengths } = params;
  const n = lengths.length;
  const im = masses.map((m) => 1 / m);

  for (let iter = 0; iter < 3; iter++) {
    const { ds } = rodVectors(s);
    const c = ds.map((d, k) => 0.5 * (dot(d, d) - lengths[k] * lengths[k]));
    if (c.every((v) => Math.abs(v) < 1e-12)) break;
    const { A } = constraintMatrices(ds, ds.map(() => [0, 0, 0]), masses);
    const mu = solveLinear(A, c.map((v) => -v));
    for (let k = 0; k < n; k++) {
      const corr = [
        ds[k][0] * mu[k] - (k + 1 < n ? ds[k + 1][0] * mu[k + 1] : 0),
        ds[k][1] * mu[k] - (k + 1 < n ? ds[k + 1][1] * mu[k + 1] : 0),
        ds[k][2] * mu[k] - (k + 1 < n ? ds[k + 1][2] * mu[k + 1] : 0),
      ];
      s.p[k] = [
        s.p[k][0] + corr[0] * im[k],
        s.p[k][1] + corr[1] * im[k],
        s.p[k][2] + corr[2] * im[k],
      ];
    }
  }

  {
    const { ds, dvs } = rodVectors(s);
    const { A, jv } = constraintMatrices(ds, dvs, masses);
    if (jv.some((v) => v !== 0)) {
      const nu = solveLinear(A, jv);
      for (let k = 0; k < n; k++) {
        const corr = [
          ds[k][0] * nu[k] - (k + 1 < n ? ds[k + 1][0] * nu[k + 1] : 0),
          ds[k][1] * nu[k] - (k + 1 < n ? ds[k + 1][1] * nu[k + 1] : 0),
          ds[k][2] * nu[k] - (k + 1 < n ? ds[k + 1][2] * nu[k + 1] : 0),
        ];
        s.v[k] = [
          s.v[k][0] - corr[0] * im[k],
          s.v[k][1] - corr[1] * im[k],
          s.v[k][2] - corr[2] * im[k],
        ];
      }
    }
  }
}

/**
 * 由角度/角速度初值构建笛卡尔状态(逐杆累积)。
 * p_k = p_{k−1} + L_k·dir(θ_k,φ_k);v_k = v_{k−1} + L_k(ωθ·eθ + ωφ·sinθ·eφ)。
 * @param {{theta:number[], phi:number[], omega:number[], omegaphi:number[]}} a
 *        弧度 / rad/s
 * @param {{masses:number[], lengths:number[]}} params
 */
export function sphericalStateFromAngles(a, params) {
  const p = [];
  const v = [];
  let prevP = [0, 0, 0];
  let prevV = [0, 0, 0];
  for (let i = 0; i < params.lengths.length; i++) {
    const l = params.lengths[i];
    const d = direction(a.theta[i], a.phi[i]);
    const pi = [prevP[0] + l * d[0], prevP[1] + l * d[1], prevP[2] + l * d[2]];
    const st = Math.sin(a.theta[i]);
    const ct = Math.cos(a.theta[i]);
    const sph = Math.sin(a.phi[i]);
    const cph = Math.cos(a.phi[i]);
    const vi = [
      prevV[0] + l * (a.omega[i] * ct * cph - a.omegaphi[i] * st * sph),
      prevV[1] + l * a.omega[i] * st,
      prevV[2] + l * (a.omega[i] * ct * sph + a.omegaphi[i] * st * cph),
    ];
    p.push(pi);
    v.push(vi);
    prevP = pi;
    prevV = vi;
  }
  return { p, v };
}

/**
 * 由笛卡尔状态反解角度/角速度(显示与对比模式用)。
 * ωφ = (v·eφ)/(l·sinθ);sinθ < 1e-8(极点)时取 0。
 */
export function sphericalAnglesFromState(s, params) {
  const theta = [];
  const phi = [];
  const omega = [];
  const omegaphi = [];
  let prevP = [0, 0, 0];
  let prevV = [0, 0, 0];
  for (let i = 0; i < params.lengths.length; i++) {
    const l = params.lengths[i];
    const d = [s.p[i][0] - prevP[0], s.p[i][1] - prevP[1], s.p[i][2] - prevP[2]];
    const dv = [s.v[i][0] - prevV[0], s.v[i][1] - prevV[1], s.v[i][2] - prevV[2]];
    const rho = Math.hypot(d[0], d[2]);
    const th = Math.atan2(rho, -d[1]);
    const ph = Math.atan2(d[2], d[0]);
    const st = Math.sin(th);
    const ct = Math.cos(th);
    const sph = Math.sin(ph);
    const cph = Math.cos(ph);
    const vDotETheta = dv[0] * ct * cph + dv[1] * st + dv[2] * ct * sph;
    const vDotEPhi = -dv[0] * sph + dv[2] * cph;
    theta.push(th);
    phi.push(ph);
    omega.push(vDotETheta / l);
    omegaphi.push(st > 1e-8 ? vDotEPhi / (l * st) : 0);
    prevP = s.p[i];
    prevV = s.v[i];
  }
  return { theta, phi, omega, omegaphi };
}

/**
 * RK4 单步积分(加速度级约束),原地更新 state,末尾做微幅稳定化。
 */
export function sphericalStep(s, params, dt) {
  const k1 = deriv(s, params);
  const k2 = deriv(offset(s, k1, dt / 2), params);
  const k3 = deriv(offset(s, k2, dt / 2), params);
  const k4 = deriv(offset(s, k3, dt), params);

  const h = dt / 6;
  const n = s.p.length;
  for (let i = 0; i < n; i++) {
    for (let c = 0; c < 3; c++) {
      s.p[i][c] += h * (k1.dp[i][c] + 2 * k2.dp[i][c] + 2 * k3.dp[i][c] + k4.dp[i][c]);
      s.v[i][c] += h * (k1.dv[i][c] + 2 * k2.dv[i][c] + 2 * k3.dv[i][c] + k4.dv[i][c]);
    }
  }
  stabilize(s, params);
  return s;
}

/**
 * 系统总能量 E = T + V(V 零点取悬挂点高度)。
 */
export function sphericalEnergy(s, params) {
  const { masses } = params;
  let kinetic = 0;
  let potential = 0;
  for (let i = 0; i < masses.length; i++) {
    kinetic += 0.5 * masses[i] * dot(s.v[i], s.v[i]);
    potential += masses[i] * G * s.p[i][1];
  }
  return kinetic + potential;
}
