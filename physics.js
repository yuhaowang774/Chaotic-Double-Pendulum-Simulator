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
