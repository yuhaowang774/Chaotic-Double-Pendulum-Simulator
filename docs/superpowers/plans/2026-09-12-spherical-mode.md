# 球面双摆模式 Implementation Plan(执行摘要)

> **For agentic workers:** 本计划为摘要版——设计依据见 `docs/superpowers/specs/2026-09-12-spherical-mode-design.md`(含 RATTLE 数学与映射公式),完整代码直接落地于各文件并由验证套件把关。步骤用 checkbox 跟踪。

**Goal:** 面板加"平面/球面"切换,球面双摆用笛卡尔坐标 + RATTLE 约束积分器实现真实三维混沌运动;平面路径零改动回归。

**Tech Stack:** 原生 ES Modules、Three.js r160(不变)、Node 内置 assert。

---

### Task 1: spherical.js + 验证套件(TDD)

**Files:** Create `tests/validate_spherical.mjs`,Create `spherical.js`

- [ ] Step 1: 写 5 项失败测试(平衡态/约束残差/能量守恒/平面极限/非平面性),运行确认 RED(`Cannot find module spherical.js`)
- [ ] Step 2: 实现 spherical.js:`direction/sphericalStateFromAngles/sphericalAnglesFromState/sphericalStep(RATTLE)/sphericalEnergy`,运行至全 PASS
- [ ] Step 3: Commit `feat: 球面双摆物理核心(笛卡尔+RATTLE)与验证套件`

### Task 2: 渲染与 UI 集成

**Files:** Modify `script.js`、`index.html`、`style.css`

- [ ] Step 1: script.js — TrailRing/TrailLine 增加 z 分量;setLine 升级 3D 端点;Pendulum 增加 mode(状态/step/isFiniteState/updateVisuals 按模式分派);physicsMode 变量 + recreatePendulums();compare.resync/deltaRadians 按模式分派,施加变量下拉按模式重建;updateDataDisplay 按模式取数;初值滑条统一"重建状态"语义;φ 滑条与模式下拉绑定
- [ ] Step 2: index.html — 物理模式下拉(摆参数区顶部)、`.spherical-only` φ/ωφ 四滑条(初始条件区)
- [ ] Step 3: style.css — `.spherical-only` 显隐样式
- [ ] Step 4: Commit `feat: 平面/球面模式切换与三维轨迹渲染`

### Task 3: 验收与收尾

- [ ] Step 1: `node tests/validate_physics.mjs` 回归全 PASS;`node tests/validate_spherical.mjs` 全 PASS
- [ ] Step 2: Commit(如有遗留)并向用户交付:改动清单 + 浏览器人工验收清单(Live Server)
