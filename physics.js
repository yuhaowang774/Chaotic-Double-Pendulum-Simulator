// 平面 N 杆链物理模块(N=1–8)—— 纯函数,无 DOM 依赖,可被 Node 直接导入验证。
// 约定:θ 从竖直向下方向量起(θ=0 为悬挂平衡态),与页面坐标 x = L·sinθ, y = −L·cosθ 一致。
//
// 运动方程(链式封闭形式,Euler–Lagrange):
//   β_ij = L_i·L_j·Σ_{k≥max(i,j)} m_k
//   M_ij = β_ij·cos(θ_i−θ_j),M α = −(c + g_vec)
//   c_i  = Σ_j β_ij·sin(θ_i−θ_j)·ω_j²      (科氏/离心)
//   g_i  = G·sinθ_i·L_i·Σ_{k≥i} m_k        (重力矩)
// N=2 时与两杆专用方程代数等价;N=1 退化为单摆。
//
// 性能:按杆数缓存 Float64Array 工作区,RK4 与高斯消元热路径零分配。

export const G = 9.8;

const workspaces = new Map();

function getWork(n) {
  let w = workspaces.get(n);
  if (!w) {
    w = {
      suffix: new Float64Array(n),
      m: new Float64Array(n * n),
      b: new Float64Array(n),
      aug: new Float64Array(n * (n + 1)),
      k: Array.from({ length: 4 }, () => ({
        dtheta: new Array(n).fill(0),
        domega: new Array(n).fill(0),
      })),
      mid: Array.from({ length: 3 }, () => ({
        theta: new Array(n).fill(0),
        omega: new Array(n).fill(0),
      })),
    };
    workspaces.set(n, w);
  }
  return w;
}

// 高斯消元(部分主元):增广 aug = [m | b],解写入 x。零分配。
// 供本模块与球面模块复用(球面约束矩阵为三对角,同样走此求解器)。
export function solveInto(m, b, n, aug, x) {
  const stride = n + 1;
  for (let r = 0; r < n; r++) {
    for (let c = 0; c < n; c++) aug[r * stride + c] = m[r * n + c];
    aug[r * stride + n] = b[r];
  }
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(aug[r * stride + col]) > Math.abs(aug[piv * stride + col])) {
        piv = r;
      }
    }
    if (piv !== col) {
      for (let c = col; c < stride; c++) {
        const tmp = aug[col * stride + c];
        aug[col * stride + c] = aug[piv * stride + c];
        aug[piv * stride + c] = tmp;
      }
    }
    const d = aug[col * stride + col];
    for (let r = col + 1; r < n; r++) {
      const f = aug[r * stride + col] / d;
      if (f !== 0) {
        for (let c = col; c < stride; c++) {
          aug[r * stride + c] -= f * aug[col * stride + c];
        }
      }
    }
  }
  for (let r = n - 1; r >= 0; r--) {
    let s = aug[r * stride + n];
    for (let c = r + 1; c < n; c++) s -= aug[r * stride + c] * x[c];
    x[r] = s / aug[r * stride + r];
  }
}

function derivativesInto(s, p, w, out) {
  const n = w.suffix.length;
  const suffix = w.suffix;
  suffix[n - 1] = p.masses[n - 1];
  for (let i = n - 2; i >= 0; i--) suffix[i] = suffix[i + 1] + p.masses[i];
  for (let i = 0; i < n; i++) {
    let rhs = -G * Math.sin(s.theta[i]) * p.lengths[i] * suffix[i];
    for (let j = 0; j < n; j++) {
      const beta = p.lengths[i] * p.lengths[j] * suffix[Math.max(i, j)];
      const delta = s.theta[i] - s.theta[j];
      w.m[i * n + j] = beta * Math.cos(delta);
      rhs -= beta * Math.sin(delta) * s.omega[j] * s.omega[j];
    }
    w.b[i] = rhs;
  }
  solveInto(w.m, w.b, n, w.aug, out.domega);
  for (let i = 0; i < n; i++) out.dtheta[i] = s.omega[i];
}

/**
 * 运动方程右端项(每次调用返回新对象,供测试/外部使用;
 * 热路径请用 rk4Step,其内部走零分配工作区)。
 */
export function derivatives(s, p) {
  const n = s.theta.length;
  const w = getWork(n);
  const out = { dtheta: new Array(n), domega: new Array(n) };
  derivativesInto(s, p, w, out);
  return out;
}

/**
 * RK4 单步积分,原地更新 state。热路径零分配。
 */
export function rk4Step(state, params, dt) {
  const n = state.theta.length;
  const w = getWork(n);
  const [k1, k2, k3, k4] = w.k;
  const [s2, s3, s4] = w.mid;

  derivativesInto(state, params, w, k1);
  for (let i = 0; i < n; i++) {
    s2.theta[i] = state.theta[i] + (dt / 2) * k1.dtheta[i];
    s2.omega[i] = state.omega[i] + (dt / 2) * k1.domega[i];
  }
  derivativesInto(s2, params, w, k2);
  for (let i = 0; i < n; i++) {
    s3.theta[i] = state.theta[i] + (dt / 2) * k2.dtheta[i];
    s3.omega[i] = state.omega[i] + (dt / 2) * k2.domega[i];
  }
  derivativesInto(s3, params, w, k3);
  for (let i = 0; i < n; i++) {
    s4.theta[i] = state.theta[i] + dt * k3.dtheta[i];
    s4.omega[i] = state.omega[i] + dt * k3.domega[i];
  }
  derivativesInto(s4, params, w, k4);

  const h = dt / 6;
  for (let i = 0; i < n; i++) {
    state.theta[i] +=
      h * (k1.dtheta[i] + 2 * k2.dtheta[i] + 2 * k3.dtheta[i] + k4.dtheta[i]);
    state.omega[i] +=
      h * (k1.domega[i] + 2 * k2.domega[i] + 2 * k3.domega[i] + k4.domega[i]);
  }
  return state;
}

/**
 * 系统总能量 E = T + V(V 零点取悬挂点高度)。
 * T = ½ Σ_ij β_ij·cos(θ_i−θ_j)·ω_i ω_j
 * V = −G·Σ_i (Σ_{k≥i} m_k)·L_i·cosθ_i
 */
export function totalEnergy(s, p) {
  const n = s.theta.length;
  const w = getWork(n);
  const suffix = w.suffix;
  suffix[n - 1] = p.masses[n - 1];
  for (let i = n - 2; i >= 0; i--) suffix[i] = suffix[i + 1] + p.masses[i];
  let kinetic = 0;
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      const beta =
        p.lengths[i] * p.lengths[j] * suffix[Math.max(i, j)];
      kinetic +=
        beta * Math.cos(s.theta[i] - s.theta[j]) * s.omega[i] * s.omega[j];
    }
  }
  kinetic *= 0.5;
  let potential = 0;
  for (let i = 0; i < n; i++) {
    potential -= G * suffix[i] * p.lengths[i] * Math.cos(s.theta[i]);
  }
  return kinetic + potential;
}
