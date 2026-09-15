# 混沌多摆模拟器

![three.js](https://img.shields.io/badge/three.js-r160-black)
![no build](https://img.shields.io/badge/build-none-brightgreen)
![tests](https://img.shields.io/badge/tests-node%20assert-blue)

基于 Three.js 与 RK4 积分的混沌多摆教学模拟器。支持 **1–8 杆平面链**、**球面 3D 双摆** 与 **混沌对比实验**，零构建、免安装依赖，一条命令即可在浏览器中运行。

从两杆到八杆、从平面到球面，直观观察确定性系统中初值的微小差异如何被放大为完全不同的运动轨迹——这就是混沌。

## 功能特性

- **双物理模式**
  - 平面模式：N 杆链在竖直平面内摆动
  - 球面模式：真实三维约束运动（笛卡尔坐标 + 约束动力学）
- **1–8 杆可调链**：每根杆的长度、质量、初始角与角速度独立可调（球面模式另含方位角 φ 与角速度 ωφ）
- **混沌对比实验**：A/B 两条链仅一个初值相差 δ，实时观察轨迹分裂；面板显示首锤/末锤的 |Δθ| 分歧读数，δ 可施加到任意杆的任意变量（θ/ω/φ/ωφ）
- **轨迹系统**：每个摆锤独立轨迹，留存时长 0.5 秒–10 分钟对数可调；环形缓冲 + 整段拷贝，数万采样点流畅渲染
- **实时数据**：各杆角度/角速度、系统总能量、运行时间
- **播放控制**：播放/暂停、重置、0.1–10× 倍速、一键随机初始值
- **镜头**：轨道相机（拖拽旋转/滚轮缩放），空闲 3 秒自动环绕，可关闭
- **界面分级**：简单模式仅保留核心操作，高级模式展开全部参数
- **数值防护**：状态发散自动暂停并提示，不会静默黑屏
- **SpaceX 风格界面**：纯黑底、高对比排版、锐利矩形，色彩只用于摆体与轨迹

## 快速开始

### 环境要求

- Node.js ≥ 20.6（用于内置预览服务器与测试；页面本身无需任何构建）
- 支持 Import Maps 与 WebGL 的现代浏览器（Chrome/Edge 89+、Firefox 108+、Safari 16.4+）

### 运行

```bash
node server.js
```

浏览器访问 <http://localhost:8642/>。

也可以使用任意静态服务器（如 VS Code Live Server、`npx serve`）。注意：页面使用 ES 模块，必须通过 HTTP 访问，直接双击 `index.html`（`file://` 协议）无法运行。

Three.js r160 通过 jsDelivr importmap 加载，无需安装任何 npm 依赖。

## 使用指南

| 控件 | 说明 |
|---|---|
| 物理模式 | 平面 / 球面 3D，切换时重建两摆 |
| 杆数 | 1–8，实时增减杆链 |
| 摆参数（高级） | 每杆 L/m 滑条，播放中也可即时生效 |
| 初始条件（高级） | 每杆 θ/ω（球面另含 φ/ωφ），暂停时可调 |
| 随机初始值 | 一键生成随机初值 |
| 显示 / 轨迹留存 | 轨迹开关与留存时长（0.5s–600s 对数滑条） |
| 播放 / 重置 / 速度 | 播放控制与 0.1–10× 倍速 |
| 镜头 | 空闲自动环绕开关；拖拽旋转、滚轮缩放 |
| 面板折叠 | 面板顶部「收起」按钮完全收起，画布左上角「展开面板」按钮恢复 |
| 混沌对比 | 启用对比摆 B、δ 差值、施加变量、重新同步 |
| 实时数据 | 各杆 θ/ω、总能量 E、运行时间、\|Δθ\| 分歧读数 |

## 物理与数值实现

### 平面 N 杆链（`physics.js`）

θ 从竖直向下量起（θ=0 为悬挂平衡态）。Euler–Lagrange 得到的封闭形式：

$$\beta_{ij} = L_i L_j \sum_{k \ge \max(i,j)} m_k$$

$$M_{ij} = \beta_{ij}\cos(\theta_i - \theta_j), \qquad M\alpha = -(c + g)$$

$$c_i = \sum_j \beta_{ij}\sin(\theta_i - \theta_j)\,\omega_j^2, \qquad g_i = G\sin\theta_i L_i \sum_{k \ge i} m_k$$

N=2 时与经典双摆方程代数等价，N=1 退化为单摆。能量：

$$T = \frac{1}{2}\sum_{ij} \beta_{ij}\cos(\theta_i-\theta_j)\,\omega_i\omega_j, \qquad V = -G\sum_i \Big(\sum_{k\ge i} m_k\Big) L_i \cos\theta_i$$

### 球面 3D 模式（`spherical.js`）

状态为各锤的笛卡尔位置/速度，约束取半式 $c_k = \frac{1}{2}(|p_k - p_{k-1}|^2 - L_k^2)$（$p_0$ 为悬挂点）。加速度由指标-1 约束方程（saddle 点形式）解析求解：

$$(JM^{-1}J^{T})\lambda = JM^{-1}F + \gamma, \qquad a_k = (0,-G,0) - \frac{\lambda_k d_k - \lambda_{k+1} d_{k+1}}{m_k}$$

系数矩阵为三对角，高斯消元求解；RK4 每步后做微幅稳定化（位置牛顿投影 + 去径向速度），约束违反量维持约 1e-12 量级。

角度约定：$\text{dir}(\theta,\varphi) = (\sin\theta\cos\varphi,\ -\cos\theta,\ \sin\theta\sin\varphi)$，φ=0、ωφ=0 时与平面模式逐点重合。

### 数值方案与性能

- RK4 固定步长 dt = 0.002 s，按帧时长补子步（每帧上限 100 步，防止后台标签页返回时长时间冻结）
- 轨迹按固定模拟时间间隔采点（0.008 s），与帧率、倍速无关
- 两个物理模块均为纯函数、无 DOM 依赖，按杆数缓存 Float64Array 工作区，积分热路径零分配

### 精度（由测试套件实测）

- 平面 / 球面能量守恒：60 秒相对漂移 < 1e-6（dt = 0.0005）
- 球面约束残差：60 秒后 < 1e-9
- 小角度极限周期与理论值 $2\pi\sqrt{L/g}$ 偏差 < 2%

## 项目结构

```
Chaotic Double Pendulum Simulator/
├── index.html                 页面骨架与控制面板
├── style.css                  SpaceX 风格设计系统
├── script.js                  Three.js 场景、摆体渲染、UI 绑定、主循环、对比管理
├── physics.js                 平面 N 杆链物理（运动方程 / RK4 / 能量）
├── spherical.js               球面 3D 物理（约束动力学 / 稳定化）
├── server.js                  本地预览服务器（端口 8642）
├── tests/
│   ├── validate_physics.mjs   平面物理验证
│   ├── validate_spherical.mjs 球面物理验证
│   ├── smoke.mjs              页面冒烟测试（DOM / Three 桩）
│   └── stubs/                 测试桩与模块加载器
├── docs/superpowers/          设计文档与实施计划
└── DESIGN_SPEC.zh-CN.md       界面设计系统规范
```

## 测试

```bash
# 平面物理验证
node tests/validate_physics.mjs

# 球面物理验证
node tests/validate_spherical.mjs

# 页面冒烟测试（模拟浏览器完整链路）
node --import ./tests/stubs/register.mjs tests/smoke.mjs
```

Windows 下可直接双击 `运行全部测试.cmd` 一键运行三套测试。

测试基于 Node 内置 `node:assert`，无任何测试框架依赖。

## 设计文档

- `docs/superpowers/specs/`：教学升级、球面模式、N 杆链三份设计规格（含完整数学推导与验收标准）
- `docs/superpowers/plans/`：对应的实施计划
- `DESIGN_SPEC.zh-CN.md`：SpaceX 风格界面设计系统

## 技术栈

- 原生 ES Modules + HTML/CSS：零构建、零打包、零 npm 依赖
- Three.js r160（jsDelivr importmap）+ OrbitControls
- 自研物理：Euler–Lagrange 通式 + 指标约减约束动力学 + RK4
- 自研测试：Node `node:assert` + DOM / Three.js 测试桩

## 贡献

欢迎提交 Issue 与 PR。提交前请运行全部测试并确保通过。

## License

本项目基于 [MIT License](LICENSE) 开源。
