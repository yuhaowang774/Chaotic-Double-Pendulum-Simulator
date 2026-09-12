// 球面双摆物理模块 —— 笛卡尔坐标 + 指标约减(约束加速度)+ RK4,纯函数,无 DOM 依赖。
// 坐标系与渲染一致:悬挂点在原点,y 轴向上,重力沿 −y。
// 角度约定:θ 为极角(自竖直向下方向,θ=0 悬挂),φ 为方位角(绕 y 轴)。
// dir(θ,φ) = (sinθcosφ, −cosθ, sinθsinφ):φ=0、ωφ=0 时与平面双摆 (x,y) 逐点重合。
//
// 积分方案:状态为两锤笛卡尔位置/速度(12 维),加速度由约束 saddle 点方程解析求解
//   M a = F − Jᵀλ,  J a = −γ,  γ = (|v1|², |v2−v1|²)(约束二阶导数项),
// 即 (J M⁻¹ Jᵀ)λ = J M⁻¹ F + γ 后得 a。RK4 积分,每步后做微幅稳定化
// (位置牛顿投影 + 去径向速度,修正量 ~1e-12,不改变能量)。
// 注:曾实现 RATTLE(高斯-牛顿投影),实测能量随速度以 O(dt) 单调衰减,已弃用。

import { G } from "./physics.js";

export { G };

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function direction(theta, phi) {
  const st = Math.sin(theta);
  const ct = Math.cos(theta);
  return [st * Math.cos(phi), -ct, st * Math.sin(phi)];
}

/**
 * 由角度/角速度初值构建笛卡尔状态。
 * p1 = l1·dir(θ1,φ1);p2 = p1 + l2·dir(θ2,φ2)(第二杆自第一锤起)。
 * v = l·(θ̇·∂dir/∂θ + φ̇·sinθ·eφ),eφ = (−sinφ, 0, cosφ)。
 * @param {{theta1:number, phi1:number, omega1:number, omegaphi1:number,
 *          theta2:number, phi2:number, omega2:number, omegaphi2:number}} a
 *        弧度 / rad/s
 * @param {{m1:number, m2:number, l1:number, l2:number}} params
 */
export function sphericalStateFromAngles(a, params) {
  const { l1, l2 } = params;
  const d1 = direction(a.theta1, a.phi1);
  const d2 = direction(a.theta2, a.phi2);
  const p1 = [l1 * d1[0], l1 * d1[1], l1 * d1[2]];
  const p2 = [p1[0] + l2 * d2[0], p1[1] + l2 * d2[1], p1[2] + l2 * d2[2]];

  const st1 = Math.sin(a.theta1);
  const ct1 = Math.cos(a.theta1);
  const v1 = [
    l1 * (a.omega1 * ct1 * Math.cos(a.phi1) - a.omegaphi1 * st1 * Math.sin(a.phi1)),
    l1 * a.omega1 * st1,
    l1 * (a.omega1 * ct1 * Math.sin(a.phi1) + a.omegaphi1 * st1 * Math.cos(a.phi1)),
  ];

  const st2 = Math.sin(a.theta2);
  const ct2 = Math.cos(a.theta2);
  const sph2 = Math.sin(a.phi2);
  const cph2 = Math.cos(a.phi2);
  const v2 = [
    v1[0] + l2 * (a.omega2 * ct2 * cph2 - a.omegaphi2 * st2 * sph2),
    v1[1] + l2 * a.omega2 * st2,
    v1[2] + l2 * (a.omega2 * ct2 * sph2 + a.omegaphi2 * st2 * cph2),
  ];
  return { p1, v1, p2, v2 };
}

/**
 * 由笛卡尔状态反解角度/角速度(显示与对比模式用)。
 * ωφ = (v·eφ)/(l·sinθ);sinθ < 1e-8(极点)时取 0。
 */
export function sphericalAnglesFromState(s, params) {
  const out = {};
  for (const i of ["1", "2"]) {
    const p = s["p" + i];
    const v = s["v" + i];
    const l = i === "1" ? params.l1 : params.l2;
    const rho = Math.hypot(p[0], p[2]);
    const theta = Math.atan2(rho, -p[1]);
    const phi = Math.atan2(p[2], p[0]);
    const st = Math.sin(theta);
    const ct = Math.cos(theta);
    const sph = Math.sin(phi);
    const cph = Math.cos(phi);
    // eθ = ∂dir/∂θ(单位),eφ = (−sinφ, 0, cosφ)(单位)
    const vDotETheta = v[0] * ct * cph + v[1] * st + v[2] * ct * sph;
    const vDotEPhi = -v[0] * sph + v[2] * cph;
    out["theta" + i] = theta;
    out["phi" + i] = phi;
    out["omega" + i] = vDotETheta / l;
    out["omegaphi" + i] = st > 1e-8 ? vDotEPhi / (l * st) : 0;
  }
  return out;
}

/**
 * 约束加速度:解 (J M⁻¹ Jᵀ)λ = J M⁻¹ F + γ,返回两锤加速度 [a1, a2]。
 * J1 = [p1, 0](c1 = ½(|p1|²−l1²)),J2 = [−d, d](c2 = ½(|p2−p1|²−l2²),d = p2−p1)。
 * J M⁻¹ F = (−g·p1_y, 0);γ = (|v1|², |v2−v1|²)。
 */
function accelerations(s, params) {
  const { m1, m2 } = params;
  const d = [s.p2[0] - s.p1[0], s.p2[1] - s.p1[1], s.p2[2] - s.p1[2]];
  const dv = [s.v2[0] - s.v1[0], s.v2[1] - s.v1[1], s.v2[2] - s.v1[2]];

  const A11 = dot(s.p1, s.p1) / m1;
  const A12 = -dot(s.p1, d) / m1;
  const A22 = dot(d, d) * (1 / m1 + 1 / m2);
  const det = A11 * A22 - A12 * A12;

  const a1g = [0, -G, 0];
  const a2g = [0, -G, 0];
  if (Math.abs(det) < 1e-14) return [a1g, a2g]; // 退化保护(l→0)

  const rhs1 = -G * s.p1[1] + dot(s.v1, s.v1);
  const rhs2 = dot(dv, dv);
  const lam1 = (rhs1 * A22 - rhs2 * A12) / det;
  const lam2 = (A11 * rhs2 - A12 * rhs1) / det;

  const a1 = [
    a1g[0] - (lam1 * s.p1[0] - lam2 * d[0]) / m1,
    a1g[1] - (lam1 * s.p1[1] - lam2 * d[1]) / m1,
    a1g[2] - (lam1 * s.p1[2] - lam2 * d[2]) / m1,
  ];
  const a2 = [
    a2g[0] - (lam2 * d[0]) / m2,
    a2g[1] - (lam2 * d[1]) / m2,
    a2g[2] - (lam2 * d[2]) / m2,
  ];
  return [a1, a2];
}

function deriv(s, params) {
  const [a1, a2] = accelerations(s, params);
  return {
    dp1: [s.v1[0], s.v1[1], s.v1[2]],
    dv1: a1,
    dp2: [s.v2[0], s.v2[1], s.v2[2]],
    dv2: a2,
  };
}

function offset(s, k, h) {
  return {
    p1: [s.p1[0] + h * k.dp1[0], s.p1[1] + h * k.dp1[1], s.p1[2] + h * k.dp1[2]],
    v1: [s.v1[0] + h * k.dv1[0], s.v1[1] + h * k.dv1[1], s.v1[2] + h * k.dv1[2]],
    p2: [s.p2[0] + h * k.dp2[0], s.p2[1] + h * k.dp2[1], s.p2[2] + h * k.dp2[2]],
    v2: [s.v2[0] + h * k.dv2[0], s.v2[1] + h * k.dv2[1], s.v2[2] + h * k.dv2[2]],
  };
}

// 微幅稳定化:位置牛顿投影(不修正速度)+ 去径向速度。每步违反量 ~1e-12。
function stabilize(s, params) {
  const { m1, m2, l1, l2 } = params;
  const im1 = 1 / m1;
  const im2 = 1 / m2;

  for (let iter = 0; iter < 3; iter++) {
    const c1 = 0.5 * (dot(s.p1, s.p1) - l1 * l1);
    const d = [s.p2[0] - s.p1[0], s.p2[1] - s.p1[1], s.p2[2] - s.p1[2]];
    const c2 = 0.5 * (dot(d, d) - l2 * l2);
    if (Math.abs(c1) < 1e-12 && Math.abs(c2) < 1e-12) break;
    const A11 = dot(s.p1, s.p1) * im1;
    const A12 = -dot(s.p1, d) * im1;
    const A22 = dot(d, d) * (im1 + im2);
    const det = A11 * A22 - A12 * A12;
    if (Math.abs(det) < 1e-14) break;
    const mu1 = (-c1 * A22 + c2 * A12) / det;
    const mu2 = (c1 * A12 - c2 * A11) / det;
    for (let k = 0; k < 3; k++) {
      s.p1[k] += (mu1 * s.p1[k] - mu2 * d[k]) * im1;
      s.p2[k] += mu2 * d[k] * im2;
    }
  }

  {
    const d = [s.p2[0] - s.p1[0], s.p2[1] - s.p1[1], s.p2[2] - s.p1[2]];
    const A11 = dot(s.p1, s.p1) * im1;
    const A12 = -dot(s.p1, d) * im1;
    const A22 = dot(d, d) * (im1 + im2);
    const det = A11 * A22 - A12 * A12;
    if (Math.abs(det) > 1e-14) {
      const dv = [s.v2[0] - s.v1[0], s.v2[1] - s.v1[1], s.v2[2] - s.v1[2]];
      const b1 = dot(s.p1, s.v1);
      const b2 = dot(d, dv);
      const nu1 = (b1 * A22 - b2 * A12) / det;
      const nu2 = (A11 * b2 - A12 * b1) / det;
      for (let k = 0; k < 3; k++) {
        s.v1[k] -= (nu1 * s.p1[k] - nu2 * d[k]) * im1;
        s.v2[k] -= nu2 * d[k] * im2;
      }
    }
  }
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
  const fields = ["p1", "v1", "p2", "v2"];
  const comps = ["dp1", "dv1", "dp2", "dv2"];
  for (let f = 0; f < 4; f++) {
    const target = s[fields[f]];
    for (let k = 0; k < 3; k++) {
      target[k] +=
        h * (k1[comps[f]][k] + 2 * k2[comps[f]][k] + 2 * k3[comps[f]][k] + k4[comps[f]][k]);
    }
  }
  stabilize(s, params);
  return s;
}

/**
 * 系统总能量 E = T + V(V 零点取悬挂点高度)。
 */
export function sphericalEnergy(s, params) {
  const { m1, m2 } = params;
  const kinetic = 0.5 * m1 * dot(s.v1, s.v1) + 0.5 * m2 * dot(s.v2, s.v2);
  const potential = m1 * G * s.p1[1] + m2 * G * s.p2[1];
  return kinetic + potential;
}
