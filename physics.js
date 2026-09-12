// 平面 N 杆链物理模块(N=1–8)—— 纯函数,无 DOM 依赖,可被 Node 直接导入验证。
// 约定:θ 从竖直向下方向量起(θ=0 为悬挂平衡态),与页面坐标 x = L·sinθ, y = −L·cosθ 一致。
//
// 运动方程(链式封闭形式,Euler–Lagrange):
//   β_ij = L_i·L_j·Σ_{k≥max(i,j)} m_k
//   M_ij = β_ij·cos(θ_i−θ_j),M α = −(c + g_vec)
//   c_i  = Σ_j β_ij·sin(θ_i−θ_j)·ω_j²      (科氏/离心)
//   g_i  = G·sinθ_i·L_i·Σ_{k≥i} m_k        (重力矩)
// N=2 时与两杆专用方程代数等价;N=1 退化为单摆。

export const G = 9.8;

// 高斯消元求线方程(部分主元);M 需非奇异(质量与杆长均为正时动能矩阵正定)。
export function solveLinear(M, b) {
  const n = b.length;
  const A = M.map((row, i) => [...row, b[i]]);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(A[r][col]) > Math.abs(A[piv][col])) piv = r;
    }
    [A[col], A[piv]] = [A[piv], A[col]];
    const d = A[col][col];
    if (Math.abs(d) < 1e-14) throw new Error("线性方程组奇异");
    for (let r = col + 1; r < n; r++) {
      const f = A[r][col] / d;
      for (let c = col; c <= n; c++) A[r][c] -= f * A[col][c];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = A[r][n];
    for (let c = r + 1; c < n; c++) s -= A[r][c] * x[c];
    x[r] = s / A[r][r];
  }
  return x;
}

function suffixMasses(p) {
  const n = p.masses.length;
  const suffix = new Array(n);
  suffix[n - 1] = p.masses[n - 1];
  for (let i = n - 2; i >= 0; i--) suffix[i] = suffix[i + 1] + p.masses[i];
  return suffix;
}

/**
 * 运动方程右端项。
 * @param {{theta:number[], omega:number[]}} s 状态(弧度 / rad/s)
 * @param {{masses:number[], lengths:number[]}} p 参数(kg / m)
 * @returns {{dtheta:number[], domega:number[]}} dtheta = ω,domega = 角加速度
 */
export function derivatives(s, p) {
  const n = s.theta.length;
  const suffix = suffixMasses(p);
  const M = [];
  const b = [];
  for (let i = 0; i < n; i++) {
    const row = new Array(n);
    let rhs = -G * Math.sin(s.theta[i]) * p.lengths[i] * suffix[i];
    for (let j = 0; j < n; j++) {
      const beta = p.lengths[i] * p.lengths[j] * suffix[Math.max(i, j)];
      const delta = s.theta[i] - s.theta[j];
      row[j] = beta * Math.cos(delta);
      rhs -= beta * Math.sin(delta) * s.omega[j] * s.omega[j];
    }
    M.push(row);
    b.push(rhs);
  }
  return { dtheta: [...s.omega], domega: solveLinear(M, b) };
}

/**
 * RK4 单步积分,原地更新 state。
 */
export function rk4Step(state, params, dt) {
  const k1 = derivatives(state, params);
  const mid = (k, h) => ({
    theta: state.theta.map((v, i) => v + h * k.dtheta[i]),
    omega: state.omega.map((v, i) => v + h * k.domega[i]),
  });
  const k2 = derivatives(mid(k1, dt / 2), params);
  const k3 = derivatives(mid(k2, dt / 2), params);
  const k4 = derivatives(mid(k3, dt), params);

  const h = dt / 6;
  state.theta = state.theta.map(
    (v, i) => v + h * (k1.dtheta[i] + 2 * k2.dtheta[i] + 2 * k3.dtheta[i] + k4.dtheta[i]),
  );
  state.omega = state.omega.map(
    (v, i) => v + h * (k1.domega[i] + 2 * k2.domega[i] + 2 * k3.domega[i] + k4.domega[i]),
  );
  return state;
}

/**
 * 系统总能量 E = T + V(V 零点取悬挂点高度)。
 * T = ½ Σ_ij β_ij·cos(θ_i−θ_j)·ω_i ω_j
 * V = −G·Σ_i (Σ_{k≥i} m_k)·L_i·cosθ_i
 */
export function totalEnergy(s, p) {
  const n = s.theta.length;
  const suffix = suffixMasses(p);
  let kinetic = 0;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      const beta = p.lengths[i] * p.lengths[j] * suffix[Math.max(i, j)];
      kinetic += beta * Math.cos(s.theta[i] - s.theta[j]) * s.omega[i] * s.omega[j];
    }
  }
  kinetic *= 0.5;
  let potential = 0;
  for (let i = 0; i < n; i++) {
    potential -= G * suffix[i] * p.lengths[i] * Math.cos(s.theta[i]);
  }
  return kinetic + potential;
}
