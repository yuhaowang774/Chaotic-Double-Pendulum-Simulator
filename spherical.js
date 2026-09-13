// 球面 N 杆链物理模块(N=1–8)—— 笛卡尔坐标 + 指标约减(约束加速度)+ RK4。
// 纯函数,无 DOM 依赖,可被 Node 直接导入验证。
// 坐标系与渲染一致:悬挂点在原点,y 轴向上,重力沿 −y。
// 角度约定:θ 为极角(自竖直向下方向,θ=0 悬挂),φ 为方位角(绕 y 轴)。
// dir(θ,φ) = (sinθcosφ, −cosθ, sinθsinφ):全 φ=0、ωφ=0 时与平面 N 杆链逐点重合。
//
// 积分方案:状态为 N 锤笛卡尔位置/速度;加速度由约束 saddle 点方程解析求解
//   M a = F − Jᵀλ,  J a = −γ,  γ_k = |v_k − v_{k−1}|²(v₀ ≡ 0),
// (J M⁻¹ Jᵀ)λ = J M⁻¹ F + γ 的系数矩阵为三对角(相邻约束共享一锤),高斯消元求解。
// RK4 积分,每步后做微幅稳定化(位置牛顿投影 + 去径向速度,修正量 ~1e-12)。
//
// 性能:按杆数缓存 Float64Array 工作区,RK4/约束求解/稳定化热路径零分配。

import { G, solveInto } from "./physics.js";

export { G };

const workspaces = new Map();

function getWork(n) {
  let w = workspaces.get(n);
  if (!w) {
    w = {
      n,
      ds: new Float64Array(n * 3),
      dvs: new Float64Array(n * 3),
      A: new Float64Array(n * n),
      b: new Float64Array(n),
      c: new Float64Array(n),
      jv: new Float64Array(n),
      lam: new Float64Array(n),
      aug: new Float64Array(n * (n + 1)),
      acc: new Float64Array(n * 3),
      im: new Float64Array(n),
      k: Array.from({ length: 4 }, () => ({
        dp: new Float64Array(n * 3),
        dv: new Float64Array(n * 3),
      })),
      off: Array.from({ length: 3 }, () => ({
        p: Array.from({ length: n }, () => [0, 0, 0]),
        v: Array.from({ length: n }, () => [0, 0, 0]),
      })),
    };
    workspaces.set(n, w);
  }
  return w;
}

function dot3(a, i, b, j) {
  return a[i * 3] * b[j * 3] + a[i * 3 + 1] * b[j * 3 + 1] + a[i * 3 + 2] * b[j * 3 + 2];
}

function rodVectorsInto(s, w) {
  const n = w.n;
  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  for (let k = 0; k < n; k++) {
    const p = s.p[k];
    const v = s.v[k];
    w.ds[k * 3] = p[0] - px;
    w.ds[k * 3 + 1] = p[1] - py;
    w.ds[k * 3 + 2] = p[2] - pz;
    w.dvs[k * 3] = v[0] - vx;
    w.dvs[k * 3 + 1] = v[1] - vy;
    w.dvs[k * 3 + 2] = v[2] - vz;
    px = p[0];
    py = p[1];
    pz = p[2];
    vx = v[0];
    vy = v[1];
    vz = v[2];
  }
}

// A = J M⁻¹ Jᵀ(三对角)+ J v;A 清零后填充
function buildAandJv(w, masses) {
  const n = w.n;
  w.A.fill(0);
  for (let k = 0; k < n; k++) {
    const ddSelf = dot3(w.ds, k, w.ds, k);
    w.A[k * n + k] = ddSelf / masses[k] + (k > 0 ? ddSelf / masses[k - 1] : 0);
    w.jv[k] = dot3(w.ds, k, w.dvs, k);
    if (k + 1 < n) {
      const off = -dot3(w.ds, k, w.ds, k + 1) / masses[k];
      w.A[k * n + k + 1] = off;
      w.A[(k + 1) * n + k] = off;
    }
  }
}

// 约束加速度写入 w.acc:a_k = (0,−G,0) − (λ_k d_k − λ_{k+1} d_{k+1})/m_k
function accelerationsInto(s, params, w) {
  const n = w.n;
  const { masses } = params;
  rodVectorsInto(s, w);
  buildAandJv(w, masses);
  for (let k = 0; k < n; k++) {
    const gamma = dot3(w.dvs, k, w.dvs, k);
    w.b[k] = (k === 0 ? -G * w.ds[k * 3 + 1] : 0) + gamma;
  }
  solveInto(w.A, w.b, n, w.aug, w.lam);
  for (let k = 0; k < n; k++) {
    let cx = w.ds[k * 3] * w.lam[k];
    let cy = w.ds[k * 3 + 1] * w.lam[k];
    let cz = w.ds[k * 3 + 2] * w.lam[k];
    if (k + 1 < n) {
      cx -= w.ds[(k + 1) * 3] * w.lam[k + 1];
      cy -= w.ds[(k + 1) * 3 + 1] * w.lam[k + 1];
      cz -= w.ds[(k + 1) * 3 + 2] * w.lam[k + 1];
    }
    w.acc[k * 3] = -cx / masses[k];
    w.acc[k * 3 + 1] = -G - cy / masses[k];
    w.acc[k * 3 + 2] = -cz / masses[k];
  }
}

function derivInto(s, params, w, k) {
  accelerationsInto(s, params, w);
  const n = w.n;
  for (let i = 0; i < n; i++) {
    k.dp[i * 3] = s.v[i][0];
    k.dp[i * 3 + 1] = s.v[i][1];
    k.dp[i * 3 + 2] = s.v[i][2];
    k.dv[i * 3] = w.acc[i * 3];
    k.dv[i * 3 + 1] = w.acc[i * 3 + 1];
    k.dv[i * 3 + 2] = w.acc[i * 3 + 2];
  }
}

function offsetInto(s, k, h, off) {
  const n = off.p.length;
  for (let i = 0; i < n; i++) {
    off.p[i][0] = s.p[i][0] + h * k.dp[i * 3];
    off.p[i][1] = s.p[i][1] + h * k.dp[i * 3 + 1];
    off.p[i][2] = s.p[i][2] + h * k.dp[i * 3 + 2];
    off.v[i][0] = s.v[i][0] + h * k.dv[i * 3];
    off.v[i][1] = s.v[i][1] + h * k.dv[i * 3 + 1];
    off.v[i][2] = s.v[i][2] + h * k.dv[i * 3 + 2];
  }
}

// 微幅稳定化:位置牛顿投影(不修正速度)+ 去径向速度。每步违反量 ~1e-12。
function stabilizeInto(s, params, w) {
  const n = w.n;
  const { masses, lengths } = params;
  const im = w.im;
  for (let k = 0; k < n; k++) im[k] = 1 / masses[k];

  for (let iter = 0; iter < 3; iter++) {
    rodVectorsInto(s, w);
    let converged = true;
    for (let k = 0; k < n; k++) {
      const dd = dot3(w.ds, k, w.ds, k);
      // 存负值:c' = ½(L²−|d|²) = −c,使 solveInto 直接解 A·μ = c'
      w.c[k] = 0.5 * (lengths[k] * lengths[k] - dd);
      if (Math.abs(w.c[k]) >= 1e-12) converged = false;
    }
    if (converged) break;
    buildAandJv(w, masses);
    solveInto(w.A, w.c, n, w.aug, w.lam);
    for (let k = 0; k < n; k++) {
      const p = s.p[k];
      p[0] += (w.ds[k * 3] * w.lam[k] - (k + 1 < n ? w.ds[(k + 1) * 3] * w.lam[k + 1] : 0)) * im[k];
      p[1] += (w.ds[k * 3 + 1] * w.lam[k] - (k + 1 < n ? w.ds[(k + 1) * 3 + 1] * w.lam[k + 1] : 0)) * im[k];
      p[2] += (w.ds[k * 3 + 2] * w.lam[k] - (k + 1 < n ? w.ds[(k + 1) * 3 + 2] * w.lam[k + 1] : 0)) * im[k];
    }
  }

  rodVectorsInto(s, w);
  buildAandJv(w, masses);
  for (let k = 0; k < n; k++) w.b[k] = w.jv[k];
  solveInto(w.A, w.b, n, w.aug, w.lam);
  for (let k = 0; k < n; k++) {
    const v = s.v[k];
    v[0] -= (w.ds[k * 3] * w.lam[k] - (k + 1 < n ? w.ds[(k + 1) * 3] * w.lam[k + 1] : 0)) * im[k];
    v[1] -= (w.ds[k * 3 + 1] * w.lam[k] - (k + 1 < n ? w.ds[(k + 1) * 3 + 1] * w.lam[k + 1] : 0)) * im[k];
    v[2] -= (w.ds[k * 3 + 2] * w.lam[k] - (k + 1 < n ? w.ds[(k + 1) * 3 + 2] * w.lam[k + 1] : 0)) * im[k];
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
    const st = Math.sin(a.theta[i]);
    const ct = Math.cos(a.theta[i]);
    const sph = Math.sin(a.phi[i]);
    const cph = Math.cos(a.phi[i]);
    const di = [st * cph, -ct, st * sph];
    p.push([
      prevP[0] + l * di[0],
      prevP[1] + l * di[1],
      prevP[2] + l * di[2],
    ]);
    v.push([
      prevV[0] + l * (a.omega[i] * ct * cph - a.omegaphi[i] * st * sph),
      prevV[1] + l * a.omega[i] * st,
      prevV[2] + l * (a.omega[i] * ct * sph + a.omegaphi[i] * st * cph),
    ]);
    prevP = p[i];
    prevV = v[i];
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
 * 热路径零分配。
 */
export function sphericalStep(s, params, dt) {
  const n = params.lengths.length;
  const w = getWork(n);
  const [k1, k2, k3, k4] = w.k;
  const [o1, o2, o3] = w.off;
  const h = dt / 6;

  derivInto(s, params, w, k1);
  offsetInto(s, k1, dt / 2, o1);
  derivInto(o1, params, w, k2);
  offsetInto(s, k2, dt / 2, o2);
  derivInto(o2, params, w, k3);
  offsetInto(s, k3, dt, o3);
  derivInto(o3, params, w, k4);

  for (let i = 0; i < n; i++) {
    for (let c = 0; c < 3; c++) {
      s.p[i][c] += h * (k1.dp[i * 3 + c] + 2 * k2.dp[i * 3 + c] + 2 * k3.dp[i * 3 + c] + k4.dp[i * 3 + c]);
      s.v[i][c] += h * (k1.dv[i * 3 + c] + 2 * k2.dv[i * 3 + c] + 2 * k3.dv[i * 3 + c] + k4.dv[i * 3 + c]);
    }
  }
  stabilizeInto(s, params, w);
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
    const v = s.v[i];
    kinetic += 0.5 * masses[i] * (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    potential += masses[i] * G * s.p[i][1];
  }
  return kinetic + potential;
}
