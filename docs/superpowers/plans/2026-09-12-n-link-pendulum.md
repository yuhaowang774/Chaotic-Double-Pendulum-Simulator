# 多杆链(N=1–8)Implementation Plan(执行摘要)

> 设计依据:`docs/superpowers/specs/2026-09-12-n-link-pendulum-design.md`(含平面通式与三对角鞍点推导)。完整代码直接落地,由两套验证套件把关;步骤用 checkbox 跟踪。

**Goal:** 两套物理与全部 UI 从固定 2 杆泛化为 N 杆(1–8),杆数运行时可调,每杆参数独立可调,全部锤显示轨迹。

---

### Task 1: 平面通式(physics.js)TDD

- [ ] Step 1: 重写 tests/validate_physics.mjs 为数组状态 + N=3 用例(RED)
- [ ] Step 2: physics.js 重写:derivatives(β/M/C/g 通式 + N×N 解线方程)、totalEnergy、rk4(数组)、通用小线性求解器;运行至 GREEN
- [ ] Step 3: Commit `feat: 平面 N 杆链通式(1-8 杆)与验证`

### Task 2: 球面通式(spherical.js)TDD

- [ ] Step 1: 重写 tests/validate_spherical.mjs 为数组状态 + N=3 用例(RED)
- [ ] Step 2: spherical.js 参数化:状态 {p[],v[]}、三对角鞍点方程、逐杆角度映射、稳定化泛化;运行至 GREEN
- [ ] Step 3: Commit `feat: 球面 N 杆链参数化与验证`

### Task 3: UI 动态化

- [ ] Step 1: script.js — params 数组化 + linkCount;动态滑条生成(每杆 L/m/θ/ω/φ/ωφ);Pendulum/轨迹调色板/数据面板/对比模式全部参数化;杆数滑条与模式切换联动
- [ ] Step 2: index.html 骨架化(动态容器替换固定滑条);style.css 卡片网格与球面滑条显隐
- [ ] Step 3: node --check + 两套套件回归
- [ ] Step 4: Commit `feat: 多杆链 UI(动态滑条/全锤轨迹/对比模式参数化)`

### Task 4: 交付

- [ ] 浏览器人工验收清单移交用户;预览服务器已在后台(8642)
