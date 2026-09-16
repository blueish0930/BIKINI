# BIKINI

非官方 Blender **5.3** 便携包（Windows x64），跟踪官方每日版并叠加自用功能。

- 文档：[主页](https://blueish0930.github.io/BIKINI/) · [手册](https://blueish0930.github.io/BIKINI/manual.html) · [更新日志](https://blueish0930.github.io/BIKINI/changelog.html) · 本地 `docs/index.html`
- 源码：[projects.blender.org/blueish/BIKINI](https://projects.blender.org/blueish/BIKINI)
- 站点 / 发布：[github.com/blueish0930/BIKINI](https://github.com/blueish0930/BIKINI)

与 Blender Foundation 无关，未做官方签名。**不是**官方 Blender。

## 使用

1. **整包解压**，不要只拷 `blender.exe`。
2. 运行同目录的 `blender.exe`。
3. 下列内容必须留在 exe 旁边：`5.3/`、`blender.crt/`、`blender.shared/`、`license/`、`bf_intern_*.dll`、`python3.dll` / `python313.dll`；若启用 DLSS，还需保留 `nvngx_dlssd.dll`（根目录与 `5.3/scripts/addons_core/cycles/` 各有一份）。

详见 `BIKINI_BUILD_INFO.txt`。节点参数见手册。

---

## 更新（新 → 旧）

### 2026-09 · Cycles 光子焦散（SPPM）

Cycles 增加 **Photon Caustics**（随机渐进光子映射）。路径：渲染属性 → Light Paths → Caustics。默认关闭；打开后由光子图提供锐利焦散，可与 DLSS 一起用。

**来源（请保留署名）**

本功能移植自 **CyclesPlus / Caustica**，不是 BIKINI 从零实现的求解器。

| 项 | 说明 |
|----|------|
| 上游仓库 | [Caustica_Blender](https://github.com/geiniyiquan250/Caustica_Blender)（水梦 / Shuimeng，GitHub `geiniyiquan250`；Blender 5.2.0 LTS 个人定制版） |
| 功能标记 | 源码 `SPDX-FileCopyrightText: 2026 CyclesPlus`，文件内 `CyclesPlus Begin/End` |
| 许可证 | **Apache-2.0**（与官方 Cycles 相同；可与 GPL-3 整包并存，见下方许可证） |
| 算法 | Stochastic Progressive Photon Mapping — Hachisuka & Jensen, *SIGGRAPH Asia 2009* |

BIKINI 侧只做了 5.3 API 适配、与时序 DLSS 的 1 spp 缓冲对齐，以及路径追踪 / 光子图的分区（避免同一条焦散被加两遍）。体积焦散、OptiX 独立光子管线未整包并入。

使用时请在 About / 文档中保留对 CyclesPlus / Caustica 与上述论文的致谢。不要把本功能写成 BIKINI 原创焦散求解器。

### v3

相对 v2 的下一批。

**渲染**

- Cycles 合并 **DLSS 4.5** 降噪（需 NVIDIA GPU；二进制闭源，见下方协议）
- 合成器 Denoise 节点增加 DLSS 4.5

**几何节点**

- Curvature 增加 Total Curvature 输出
- Wrangle 节点（VEX、数组属性）
- 补上 SDF Shape、Fractal
- Modal Tool 模态工具
- Flip Solver（WIP）

**着色器**

- Parallax Occlusion、Billboard
- Wrangle、ddx / ddy（Derivative）

**界面**

- 节点编辑器缩略图
- 叠加层显示节点原名称
- 节点组 Lock
- 折线连线样式（angled links）
- 节点编辑器 Coordinate 背景
- Noise / Voronoi 增加 tiling
- Layered texture painting、GTE refresh

### v2

**几何节点**

- CGAL 计算几何（Add → Bikini → Lib → CGAL）
- Box Engine（Box2D 二维、Box3D 三维）、Jolt 三维刚体
- Set Group Input Default、Guide Geometry、Attribute Transfer
- Get / Set Vector Component、Get / Set Matrix Component
- Make It Stand、Optimal Transport

**GPU Texture Editor**

- Camera View、Island UV、Island Padding
- Portal、FFT、Import Geo、Geo SDF
- Render Material、ShaderToy
- Terrain / Erosion（盖亚风格：山体图元、水力/热力侵蚀、雪/河/海、阶地、坡度遮罩、卫星着色）

**着色器**

- Portal、字符串节点

**物体编辑器**

- 用节点创建 / 引用物体，改变换、可见性、材质槽、修改器、父级、删除

**界面**

- G 只移动，不插入连线
- 改名高亮
- Shift+LMB 改接口数据类型（替代 v1 的 Ctrl+LMB）
- 不同编辑器之间复制粘贴节点组
- Dirty 评估（不再整树重煮）；Ctrl 点击预览
- 电子表格 Group / Attribute Filter
- Drag Search 建组不再整树重评估
- 3D 视口纹理绘制模式可画 8K（不是 GTE）

### v1

相对官方 5.3 第一批改动。

**几何节点**

- String 属性、Portal、几何 Clip、设置默认闭包
- Loop 细分、Nearest Neighbours、Debug、Time Shift
- 模拟区 Cache Limit；Repeat Zone Break
- Sparse Matrix Math、Linear Solver、Mesh Laplace
- Object Info Seed
- Select / Edit Element
- Heat Geodesic、切向场、Gradient / Divergence
- Instant Meshes、QuadWild、Triangle Remesh
- Voronoi 破碎、Delaunay 3D、图染色
- 曲线求交、Write at Index、RBF Interpolate
- Expression（自动补全）
- 组接口 enable 优化

**界面**

- 视口叠加层属性预览
- 节点组 Separator / Message；参数换行；Ctrl+LMB 改名
- 抖动拆线、一次插入多个节点
- 按住 U 拖动对齐
- 电子表格属性排序
- 组 Menu 多选，输出 List
- Ctrl+LMB 改接口数据类型，Alt+LMB 改接口形状（v2 改为 Shift+LMB 改类型）

**着色器**

- Image Socket、SDF Shape、Fractal、Expression、HLSL

**编辑器**

- Data-Block Graph
- 合成器里的 GPU Texture Editor 组节点

**GPU Texture Editor**

- Import Points、Point Stamp
- Normal ↔ Height
- Simulation / Repeat、流体输入输出
- Rasterize Geometry
- Sample / Write at Pixel、Paint、Histogram
- SDF Shape、Fractal
- Bake Image、Image Output

---

## 第三方库与署名

完整许可证文本在 `license/`（`license/license.md`、`license/spdx/`、`license/others/`）。下表只列 **BIKINI 额外引入或需要单独致谢** 的部分；官方 Blender 自带的依赖仍以 `license/license.md` 为准。

| 库 / 作品 | 版本 / 说明 | 许可证 | 用在 | 出处 |
|-----------|-------------|--------|------|------|
| **CyclesPlus / Caustica 光子焦散** | SPPM，移植自 5.2 下游 | **Apache-2.0** · © 2026 CyclesPlus | Cycles Photon Caustics | [Caustica_Blender](https://github.com/geiniyiquan250/Caustica_Blender) · 水梦 / `geiniyiquan250` |
| [CGAL](https://www.cgal.org) | 6.2 | GPL-3.0-or-later / LGPL | 几何节点 CGAL | [cgal.org](https://www.cgal.org) · [GitHub](https://github.com/CGAL/cgal) |
| [Box2D](https://box2d.org) | 3 | MIT · Erin Catto | Box Engine 2D | [GitHub](https://github.com/erincatto/box2d) |
| [Box3D](https://github.com/erincatto/box3d) | — | MIT · Erin Catto | Box Engine 3D | [GitHub](https://github.com/erincatto/box3d) |
| [Jolt Physics](https://github.com/jrouwe/JoltPhysics) | 5.6.0 | MIT · Jorrit Rouwe | Jolt Solver | [GitHub](https://github.com/jrouwe/JoltPhysics) |
| [Instant Meshes](https://github.com/wjakob/instant-meshes) | — | **BSD-3-Clause** · Jakob / Panozzo / Tarini / Sorkine-Hornung | Instant Meshes 重网格 | [GitHub](https://github.com/wjakob/instant-meshes) |
| [QRemeshify / QuadWild](https://github.com/ksami/QRemeshify) | 插件整包 | **GPL-3.0** | QuadWild 节点 | [GitHub](https://github.com/ksami/QRemeshify) · 见 `quadwild/README.txt` |
| [QuadriFlow](https://github.com/hjwdzh/QuadriFlow) | 27a6867 | MIT | 四边重网格 | [GitHub](https://github.com/hjwdzh/QuadriFlow) |
| [pmp-library](https://github.com/pmp-library/pmp-library) | 核心 remesh | MIT | Triangle Remesh | [GitHub](https://github.com/pmp-library/pmp-library) |
| [Voro++](https://github.com/chr1shr/voro) | — | BSD-3-Clause · LBNL | Voronoi 破碎 | [GitHub](https://github.com/chr1shr/voro) |
| [Eigen](https://gitlab.com/libeigen/eigen) | 5.x | MPL-2.0 | 稀疏线性代数 | [GitLab](https://gitlab.com/libeigen/eigen) |
| [Spectra](https://github.com/yixuan/spectra) | — | MPL-2.0 | 特征分解 | [GitHub](https://github.com/yixuan/spectra) |
| [JET](https://github.com/doyubkim/fluid-engine-dev) | fluid-engine-dev | MIT · Doyub Kim | Flip Solver | [GitHub](https://github.com/doyubkim/fluid-engine-dev) |
| [WebGL Fluid Simulation](https://github.com/PavelDoGreat/WebGL-Fluid-Simulation) | — | MIT · Pavel Dobryakov | GTE 流体参考 | [GitHub](https://github.com/PavelDoGreat/WebGL-Fluid-Simulation) |
| **NVIDIA DLSS / NGX** | 4.5（`nvngx_dlssd.dll`） | **专有（非开源）** | Cycles / 合成器 DLSS | [NVIDIA DLSS](https://github.com/NVIDIA/DLSS) · `license/others/NVIDIA-RTX-SDK.txt` |

论文 / 方法引用（实现参考，不是整库拷贝）：

- Hachisuka, T., Jensen, H. W. *Stochastic Progressive Photon Mapping.* ACM SIGGRAPH Asia 2009.
- Macklin / Müller / Bender, XPBD（PBD Solver 的方法来源；本树是独立精简实现，不是 PositionBasedDynamics 整仓）
- Jiang / Hu et al., MLS-MPM（MPM Solver 的方法来源）

源码目录（开源部分）：`extern/cgal`、`extern/box2d`、`extern/box3d`、`extern/jolt`、`extern/instant-meshes`、`extern/pmp-library`、`extern/quadriflow`、`extern/voro++`、`extern/jet`、`intern/cycles`（含 CyclesPlus 焦散，Apache-2.0）。DLSS 仅以二进制 DLL 形式附带，**无对应开源树**。

## 许可证

这是 **GPL 衍生作品**，不是 MIT 小工具。

- Blender 应用程序按 **GPL-3.0-or-later** 分发（官方说明；个别源文件 SPDX 仍为 GPL-2.0-or-later）。
- **Cycles** 本身是 **Apache-2.0**。CyclesPlus 焦散文件同为 Apache-2.0，留在 `intern/cycles`，没有改成 GPL。
- Apache-2.0 **可以**放进 GPL-3 发行包，**不能**放进纯 GPL-2 包。本构建因 CGAL、QRemeshify 等已按 GPL-3 分发，兼容。
- CGAL、QRemeshify/QuadWild 是 **GPL-3**，链进 exe 后整包按 GPL-3 分发。
- Instant Meshes 是 **BSD-3-Clause**（不是 GPL）。Box2D / Box3D / Jolt / PMP / QuadriFlow / JET 是 MIT，保留版权声明即可。
- 分发时请保留整个 `license/` 目录，**不要**声称这是官方 Blender，也**不要**把光子焦散写成 BIKINI 原创。

### 特别说明：NVIDIA DLSS 4.5（专有组件）

本构建可选包含 **NVIDIA DLSS / NGX** 运行库（`nvngx_dlssd.dll`），用于降噪等功能。请注意：

1. **不是开源软件**。DLSS / NGX 受 NVIDIA RTX SDKs License 约束，版权归 NVIDIA Corporation；与 GPL 组件**并存**，但 **DLSS 二进制本身不因同包分发而变成 GPL，也不得被“传染”为开源义务**（NVIDIA 许可证明确禁止以会导致 SDK 适用开源许可证的方式使用）。
2. **只能作为本应用的一部分、以目标代码形式分发**，不得单独拆出 `nvngx_dlssd.dll` 当作独立产品再分发；不得对 DLL 做逆向工程、去版权声明等。
3. **面向 NVIDIA GPU**。DLSS / NGX 功能依赖兼容的 NVIDIA 硬件与驱动；在非 NVIDIA 设备上相关选项可能不可用或自动回退。
4. **商标与背书**。除非另有 NVIDIA 书面许可，不得暗示本构建由 NVIDIA 赞助或背书。可按 NVIDIA 许可证要求在 About / 启动相关说明中标注使用了 DLSS / NGX。
5. **完整条款**见 `license/others/NVIDIA-RTX-SDK.txt`（NVIDIA RTX SDKs License，含 DLSS / NGX；v. March 14, 2024）。官方原文：<https://github.com/NVIDIA/DLSS/blob/main/LICENSE.txt>。商业发布前请按该许可证履行 NVIDIA 通知等义务（见补充条款中的 Notification）。

若你分发本便携包：请保留上述 DLL 与 `license/others/NVIDIA-RTX-SDK.txt`；若你**删除** DLSS DLL，则本专有条款对该副本不再适用，但其余 GPL / Apache / 第三方声明仍须保留。
